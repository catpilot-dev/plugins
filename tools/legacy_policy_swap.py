#!/usr/bin/env python3
"""Convert a legacy split driving model (driving_vision.onnx + driving_policy.onnx) into an openpilot 0.11.2
driving_supercombo.onnx by swapping policy weights inside a template supercombo.

PC-only maintainer tool. The template supercombo (e.g. the stock 0.11.2 file) supplies the graph and glue; the
legacy vision must be byte-identical to the template's vision, and the legacy policy must have the same architecture
as the template's policy. The template's policy nodes are the ones with an output named model/p_*; their float16
weights are aligned to the legacy policy's by position (consuming-op type and shape must agree) and replaced.

  python tools/legacy_policy_swap.py TEMPLATE LEGACY_VISION LEGACY_POLICY OUTPUT [--no-verify]

The last line printed is the catalog `files` object for the output (fill in the release asset URL).
"""
import argparse
import base64
import hashlib
import json
import os
import pickle
import re
import sys

import numpy as np
import onnx
from onnx import numpy_helper

PRODUCER = 'catpilot-policy-swap'
OUTPUT_NAME = 'driving_supercombo.onnx'
URL_PLACEHOLDER = '<fill in release asset URL>'
OUTPUTS = 'outputs'
RECURRENCE_STEPS = 10


class ConversionError(Exception):
  pass


def weight_sequence(model, node_filter=None):
  """(op_type, initializer_name, dims) of float16 initializers in the order their consuming nodes appear (node
  order, then input position), each listed once at its first use; only nodes where node_filter(node) is True."""
  init = {i.name: i for i in model.graph.initializer}
  seq, seen = [], set()
  for n in model.graph.node:
    if node_filter is not None and not node_filter(n):
      continue
    for x in n.input:
      if x in init and x not in seen and init[x].data_type == onnx.TensorProto.FLOAT16:
        seen.add(x)
        seq.append((n.op_type, x, tuple(init[x].dims)))
  return seq


def is_policy_node(node):
  return any(o.startswith('model/p_') for o in node.output)


def _meta(model, key):
  for p in model.metadata_props:
    if p.key == key:
      return p.value
  raise ConversionError(f'{model.graph.name or "model"} has no {key} metadata')


def _output_slices(model):
  return pickle.loads(base64.b64decode(_meta(model, 'output_slices')))


def _array_key(init):
  a = numpy_helper.to_array(init)
  return a.shape, hashlib.sha256(a.tobytes()).digest()


def check_vision_identical(template, legacy_vision):
  have = {_array_key(i) for i in template.graph.initializer if i.data_type == onnx.TensorProto.FLOAT16}
  missing = [i for i in legacy_vision.graph.initializer
             if i.data_type == onnx.TensorProto.FLOAT16 and _array_key(i) not in have]
  if missing:
    raise ConversionError(
      f'legacy vision {_meta(legacy_vision, "model_checkpoint")} differs from the template\'s vision '
      f'(template {_meta(template, "model_checkpoint")}): {len(missing)} float16 arrays have no byte-identical '
      f'counterpart, first {missing[0].name} {tuple(missing[0].dims)}')


def align_policy(template, legacy_policy):
  """template initializer name -> legacy policy initializer name, by position in the weight sequences."""
  t, s = weight_sequence(template, is_policy_node), weight_sequence(legacy_policy)
  if len(t) != len(s):
    raise ConversionError(f'policy weight count differs: template {len(t)}, legacy {len(s)}')
  for k, ((t_op, t_name, t_dims), (s_op, s_name, s_dims)) in enumerate(zip(t, s)):
    if (t_op, t_dims) != (s_op, s_dims):
      raise ConversionError(f'policy architecture differs at weight {k}: template {t_op} {t_dims} ({t_name}), '
                            f'legacy {s_op} {s_dims} ({s_name})')
  return {t_name: s_name for (_, t_name, _), (_, s_name, _) in zip(t, s)}


def swap_policy(template, legacy_vision, legacy_policy):
  check_vision_identical(template, legacy_vision)
  mapping = align_policy(template, legacy_policy)
  source = {i.name: i for i in legacy_policy.graph.initializer}
  out = onnx.ModelProto()
  out.CopyFrom(template)
  for init in out.graph.initializer:
    if init.name in mapping:
      new = source[mapping[init.name]]
      if not init.raw_data or not new.raw_data:
        raise ConversionError(f'{init.name} / {new.name}: only raw_data tensors are supported')
      init.raw_data = new.raw_data
  policy_checkpoint = _meta(legacy_policy, 'model_checkpoint')
  checkpoint = next((p for p in out.metadata_props if p.key == 'model_checkpoint'), None)
  vision = re.match(r'(.*?/\d+)/', checkpoint.value) if checkpoint is not None else None
  if vision is None:
    raise ConversionError('template model_checkpoint has no <vision>/<n>/ prefix: '
                          f'{checkpoint.value if checkpoint is not None else "(missing)"}')
  checkpoint.value = f'{vision.group(1)}/{policy_checkpoint}'
  out.producer_name = PRODUCER
  onnx.checker.check_model(out)
  return out


def _session(model):
  import onnxruntime as ort
  return ort.InferenceSession(model.SerializeToString(), providers=['CPUExecutionProvider'])


def _run(model, feed):
  return dict(zip([o.name for o in model.graph.output], _session(model).run(None, feed)))


def _random_inputs(model, rng, names=None):
  """Float inputs are drawn on the float16 grid: onnxruntime's CPU cast insertion may fold the glue's
  float32->float16 Cast into its own float16->float32 Cast in front of a policy op without a float16 kernel, so an
  off-grid input would reach the embedded policy unrounded while the standalone policy sees it rounded."""
  feed = {}
  for i in model.graph.input:
    if names is not None and i.name not in names:
      continue
    shape = [d.dim_value if d.HasField('dim_value') else 1 for d in i.type.tensor_type.shape.dim]
    dtype = onnx.helper.tensor_dtype_to_np_dtype(i.type.tensor_type.elem_type)
    if np.issubdtype(dtype, np.integer):
      feed[i.name] = rng.integers(0, 256, size=shape).astype(dtype)
    else:
      feed[i.name] = rng.standard_normal(shape).astype(np.float16).astype(dtype)
  return feed


def _policy_glue_tensors(model):
  """Non-initializer inputs of policy nodes that no policy node produces."""
  inits = {i.name for i in model.graph.initializer}
  policy = [n for n in model.graph.node if is_policy_node(n)]
  produced = {o for n in policy for o in n.output}
  return sorted({x for n in policy for x in n.input if x and x not in inits and x not in produced})


def _glue_values(converted, feed):
  glue = _policy_glue_tensors(converted)
  inferred = onnx.shape_inference.infer_shapes(converted, data_prop=True)
  value_info = {v.name: v for v in inferred.graph.value_info}
  probe = onnx.ModelProto()
  probe.CopyFrom(converted)
  for t in glue:
    if t not in value_info:
      raise ConversionError(f'cannot infer the type of policy input tensor {t}')
    probe.graph.output.append(value_info[t])
  outs = _run(probe, feed)
  return {t: outs[t] for t in glue}


def _policy_feed(glue, legacy_policy):
  """Match the glue tensors to the legacy policy's inputs by shape, cast to the legacy input dtype."""
  inputs = list(legacy_policy.graph.input)
  if len(glue) != len(inputs):
    raise ConversionError(f'template policy takes {len(glue)} glue tensors {sorted(glue)}, legacy policy {len(inputs)}')
  feed = {}
  for i in inputs:
    shape = tuple(d.dim_value for d in i.type.tensor_type.shape.dim)
    hits = [t for t, a in glue.items() if a.shape == shape]
    if len(hits) != 1:
      raise ConversionError(f'legacy policy input {i.name} {shape} matches glue tensors {hits}, need exactly one')
    feed[i.name] = glue[hits[0]].astype(onnx.helper.tensor_dtype_to_np_dtype(i.type.tensor_type.elem_type))
  return feed


def verify(template, converted, legacy_policy, seed=0):
  rng = np.random.default_rng(seed)
  feed = _random_inputs(template, rng)
  session = _session(converted)
  names = [o.name for o in converted.graph.output]
  ref, got = _run(template, feed), dict(zip(names, session.run(None, feed)))
  if set(ref) != set(got):
    raise ConversionError(f'outputs differ: template {sorted(ref)}, converted {sorted(got)}')

  policy_slices = _output_slices(legacy_policy)
  slices = _output_slices(converted)
  for key, s in slices.items():
    if key not in policy_slices and not np.array_equal(ref[OUTPUTS][..., s], got[OUTPUTS][..., s]):
      raise ConversionError(f'vision slice {key} differs from the template')
  for name in ref:
    if name.startswith('next_state_') and not np.array_equal(ref[name], got[name]):
      raise ConversionError(f'{name} differs from the template')

  standalone = _run(legacy_policy, _policy_feed(_glue_values(converted, feed), legacy_policy))[OUTPUTS]
  for key, s in policy_slices.items():
    if key not in slices:
      raise ConversionError(f'converted output_slices has no policy slice {key}')
    d = np.abs(got[OUTPUTS][..., slices[key]].astype(np.float32) - standalone[..., s].astype(np.float32))
    if not np.all(d == 0.0):
      raise ConversionError(f'policy slice {key} differs from the standalone legacy policy: max|d|={np.nanmax(d)}')

  states = {n[len('next_'):]: n for n in got if n.startswith('next_state_') and n[len('next_'):] in feed}
  cur = dict(feed)
  for step in range(RECURRENCE_STEPS):
    outs = dict(zip(names, session.run(None, cur)))
    for name, a in outs.items():
      if not np.all(np.isfinite(a)):
        raise ConversionError(f'recurrence step {step}: {name} is not finite')
    cur = _random_inputs(converted, rng, names=set(feed) - set(states))
    cur.update({s: outs[n] for s, n in states.items()})


def _sha256(path):
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    for chunk in iter(lambda: f.read(1 << 20), b''):
      h.update(chunk)
  return h.hexdigest()


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
  ap.add_argument('template', help='template driving_supercombo.onnx (e.g. stock 0.11.2)')
  ap.add_argument('legacy_vision', help='legacy driving_vision.onnx')
  ap.add_argument('legacy_policy', help='legacy driving_policy.onnx')
  ap.add_argument('output', help='converted driving_supercombo.onnx to write')
  ap.add_argument('--no-verify', action='store_true', help='skip the onnxruntime verification')
  args = ap.parse_args(argv)
  template, legacy_vision, legacy_policy = (onnx.load(p) for p in (args.template, args.legacy_vision, args.legacy_policy))
  try:
    converted = swap_policy(template, legacy_vision, legacy_policy)
    print(f'model_checkpoint {_meta(converted, "model_checkpoint")}')
    if not args.no_verify:
      verify(template, converted, legacy_policy)
      print('verify: vision slices and next_state_* identical to the template, policy slices identical to the '
            f'standalone legacy policy, {RECURRENCE_STEPS}-step recurrence finite')
  except ConversionError as e:
    print(f'error: {e}', file=sys.stderr)
    return 1
  onnx.save(converted, args.output)
  sha256, size = _sha256(args.output), os.path.getsize(args.output)
  print(f'sha256 {sha256}')
  print(f'size {size}')
  print(json.dumps({'name': OUTPUT_NAME, 'url': URL_PLACEHOLDER, 'sha256': sha256, 'size': size}))
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
