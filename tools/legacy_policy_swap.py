#!/usr/bin/env python3
"""Convert a legacy split driving model (driving_vision.onnx + driving_policy.onnx) into an openpilot 0.11.2
driving_supercombo.onnx by swapping weights inside a template supercombo.

PC-only maintainer tool. The template supercombo (e.g. the stock 0.11.2 file) supplies the graph and glue.
- Vision: every legacy float16 array must be byte-identical to a template array, except the top-level hidden-state
  head (`summarizer.*`), which a model may have retrained. The head's arrays are aligned to the template's head Gemms
  (inputs 1 and 2 of the nodes consuming model/summarizer.*.weight) by op and shape and transplanted; a head bias the
  template shares with another node is un-tied into a new model/summarizer.<layer>.bias initializer.
- Policy: the template's policy nodes are the ones with an output named model/p_*; their float16 weights are aligned
  to the legacy policy's by position (consuming-op type and shape must agree) and replaced.

  python tools/legacy_policy_swap.py TEMPLATE LEGACY_VISION LEGACY_POLICY OUTPUT [--no-verify]

The last line printed is the catalog `files` object for the output (fill in the release asset URL).
"""
import argparse
import base64
import collections
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
HIDDEN = 'hidden_state'
FEAT_Q, NEXT_FEAT_Q = 'state_feat_q', 'next_state_feat_q'
LEGACY_HEAD, TEMPLATE_HEAD = 'summarizer.', 'model/summarizer.'
RECURRENCE_STEPS = 10
FP16 = onnx.TensorProto.FLOAT16


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
      if x in init and x not in seen and init[x].data_type == FP16:
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


def _head_sequence(model, is_head):
  """[(node, input position, op_type, initializer name, dims)] for inputs 1 and 2 of the head nodes, in node order."""
  init = {i.name: i for i in model.graph.initializer}
  seq = []
  for n in model.graph.node:
    if len(n.input) > 1 and is_head(n.input[1]):
      for pos in (1, 2):
        if pos < len(n.input) and n.input[pos]:
          i = init.get(n.input[pos])
          if i is None or i.data_type != FP16:
            raise ConversionError(f'head node {n.output[0]} input {pos} ({n.input[pos]}) is not a float16 initializer')
          seq.append((n, pos, n.op_type, i.name, tuple(i.dims)))
  return seq


def _head_pairs(template, legacy_vision):
  """[(template node, input position, template initializer, legacy initializer)] for the hidden-state head."""
  def refuse(why):
    raise ConversionError(f'legacy vision {_meta(legacy_vision, "model_checkpoint")} hidden-state head does not fit the '
                          f'template {_meta(template, "model_checkpoint")}: {why}')
  t = _head_sequence(template, lambda x: x.startswith(TEMPLATE_HEAD) and x.endswith('.weight'))
  s = _head_sequence(legacy_vision, lambda x: x.startswith(LEGACY_HEAD))
  if len(t) != len(s):
    refuse(f'{len(t)} template head arrays, {len(s)} legacy')
  for k, (tt, ss) in enumerate(zip(t, s)):
    if (tt[2], tt[4]) != (ss[2], ss[4]):
      refuse(f'array {k}: template {tt[2]} {tt[4]} ({tt[3]}), legacy {ss[2]} {ss[4]} ({ss[3]})')
  covered = {ss[3] for ss in s}
  stray = [i.name for i in legacy_vision.graph.initializer
           if i.data_type == FP16 and i.name.startswith(LEGACY_HEAD) and i.name not in covered]
  if stray:
    refuse(f'{stray} are not consumed by the head Gemms')
  return [(tt[0], tt[1], tt[3], ss[3]) for tt, ss in zip(t, s)]


def check_vision_compatible(template, legacy_vision):
  """legacy initializer name -> template initializer name for the hidden-state head arrays. Every other legacy
  float16 array must have a byte-identical counterpart in the template."""
  have = {_array_key(i) for i in template.graph.initializer if i.data_type == FP16}
  missing = [i for i in legacy_vision.graph.initializer
             if i.data_type == FP16 and not i.name.startswith(LEGACY_HEAD) and _array_key(i) not in have]
  if missing:
    raise ConversionError(
      f'legacy vision {_meta(legacy_vision, "model_checkpoint")} differs from the template\'s vision '
      f'(template {_meta(template, "model_checkpoint")}): {len(missing)} float16 arrays outside the hidden-state head '
      f'have no byte-identical counterpart, first {missing[0].name} {tuple(missing[0].dims)}')
  return {legacy: tmpl for _, _, tmpl, legacy in _head_pairs(template, legacy_vision)}


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


def _replace_raw(dst, src):
  if not dst.raw_data or not src.raw_data:
    raise ConversionError(f'{dst.name} / {src.name}: only raw_data tensors are supported')
  dst.raw_data = src.raw_data


def swap_weights(template, legacy_vision, legacy_policy):
  check_vision_compatible(template, legacy_vision)
  policy = align_policy(template, legacy_policy)
  out = onnx.ModelProto()
  out.CopyFrom(template)
  init = {i.name: i for i in out.graph.initializer}

  source = {i.name: i for i in legacy_policy.graph.initializer}
  for t_name, s_name in policy.items():
    _replace_raw(init[t_name], source[s_name])

  source = {i.name: i for i in legacy_vision.graph.initializer}
  uses = collections.Counter(x for n in out.graph.node for x in n.input)
  names = set(init) | {o for n in out.graph.node for o in n.output} | {i.name for i in out.graph.input}
  for node, pos, t_name, s_name in _head_pairs(out, legacy_vision):
    src = source[s_name]
    if src.raw_data == init[t_name].raw_data:
      continue
    if uses[t_name] == 1:
      _replace_raw(init[t_name], src)
    elif pos == 1:
      raise ConversionError(f'head weight {t_name} is shared with another node; refusing to change it')
    else:
      name = node.input[1][:-len('weight')] + 'bias'
      if name in names:
        raise ConversionError(f'cannot un-tie {t_name} for {node.output[0]}: {name} already exists')
      if not src.raw_data:
        raise ConversionError(f'{src.name}: only raw_data tensors are supported')
      bias = onnx.TensorProto(name=name, data_type=src.data_type, dims=src.dims, raw_data=src.raw_data)
      out.graph.initializer.append(bias)
      node.input[2] = name
      names.add(name)

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


def _np_dtype(value_info):
  return onnx.helper.tensor_dtype_to_np_dtype(value_info.type.tensor_type.elem_type)


def _random_inputs(model, rng, names=None):
  """Float inputs are drawn on the float16 grid: onnxruntime's CPU cast insertion may fold the glue's
  float32->float16 Cast into its own float16->float32 Cast in front of a policy op without a float16 kernel, so an
  off-grid input would reach the embedded policy unrounded while the standalone policy sees it rounded."""
  feed = {}
  for i in model.graph.input:
    if names is not None and i.name not in names:
      continue
    shape = [d.dim_value if d.HasField('dim_value') else 1 for d in i.type.tensor_type.shape.dim]
    dtype = _np_dtype(i)
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


def _vision_feed_tensors(model, legacy_vision):
  """legacy vision input -> template tensor carrying the same frames. The legacy vision Casts each camera input into
  one Concat; the template's single non-policy Concat whose inputs are all Casts is its counterpart, by position."""
  inputs = {i.name for i in legacy_vision.graph.input}

  def cast_concats(m, source_ok):
    prod = {o: n for n in m.graph.node for o in n.output}
    return [[prod[x].input[0] for x in n.input] for n in m.graph.node
            if n.op_type == 'Concat' and not is_policy_node(n)
            and all(x in prod and prod[x].op_type == 'Cast' and source_ok(prod[x].input[0]) for x in n.input)]

  legacy = cast_concats(legacy_vision, lambda x: x in inputs)
  if len(legacy) != 1 or set(legacy[0]) != inputs:
    raise ConversionError(f'legacy vision inputs {sorted(inputs)} do not all feed one Cast+Concat: {legacy}')
  tmpl = [c for c in cast_concats(model, lambda x: True) if len(c) == len(legacy[0])]
  if len(tmpl) != 1:
    raise ConversionError(f'template has {len(tmpl)} Cast+Concat vision entries, need exactly one: {tmpl}')
  return dict(zip(legacy[0], tmpl[0]))


def _capture(model, feed, tensors):
  """Values of intermediate tensors on feed; graph inputs come from the feed, the rest become extra outputs."""
  outputs = {o.name for o in model.graph.output}
  extra = [t for t in tensors if t not in feed and t not in outputs]
  value_info = {v.name: v for v in onnx.shape_inference.infer_shapes(model, data_prop=True).graph.value_info}
  probe = onnx.ModelProto()
  probe.CopyFrom(model)
  for t in extra:
    if t not in value_info:
      raise ConversionError(f'cannot infer the type of tensor {t}')
    probe.graph.output.append(value_info[t])
  outs = _run(probe, feed)
  return {t: feed[t] if t in feed else outs[t] for t in tensors}


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
    feed[i.name] = glue[hits[0]].astype(_np_dtype(i))
  return feed


def _max_diff(a, b):
  """None when a and b are equal element for element (NaN counts as a difference), else max|a - b|."""
  if a.shape != b.shape:
    raise ConversionError(f'cannot compare shapes {a.shape} and {b.shape}')
  d = np.abs(a.astype(np.float32) - b.astype(np.float32))
  if np.all(d == 0.0):
    return None
  return float('nan') if np.all(np.isnan(d)) else float(np.nanmax(d))


def verify(template, converted, legacy_vision, legacy_policy, seed=0):
  """On the same random inputs, with tolerance 0.0:
  - every vision slice except hidden_state equals the template's (only the head may change);
  - next_state_feat_q is state_feat_q shifted by one with the converted hidden_state appended; every other
    next_state_* equals the template's;
  - the whole vision range equals the standalone legacy vision run on the glue's camera tensors;
  - the policy slices equal the standalone legacy policy run on the glue's policy-input tensors;
  - a 10-step recurrence stays finite."""
  rng = np.random.default_rng(seed)
  feed = _random_inputs(template, rng)
  session = _session(converted)
  names = [o.name for o in converted.graph.output]
  ref, got = _run(template, feed), dict(zip(names, session.run(None, feed)))
  if set(ref) != set(got):
    raise ConversionError(f'outputs differ: template {sorted(ref)}, converted {sorted(got)}')

  policy_slices, vision_slices, slices = (_output_slices(m) for m in (legacy_policy, legacy_vision, converted))
  vision_keys = [k for k in slices if k not in policy_slices]
  for key in vision_keys:
    if key != HIDDEN and _max_diff(ref[OUTPUTS][..., slices[key]], got[OUTPUTS][..., slices[key]]) is not None:
      raise ConversionError(f'vision slice {key} differs from the template (only {HIDDEN} may change)')
  for name in ref:
    if not name.startswith('next_state_'):
      continue
    if name == NEXT_FEAT_Q and FEAT_Q in feed and HIDDEN in slices:
      q = got[name]
      ok = np.array_equal(q[:-1], feed[FEAT_Q][1:]) and \
        np.array_equal(q[-1].ravel(), got[OUTPUTS][..., slices[HIDDEN]].ravel())
    else:
      ok = np.array_equal(ref[name], got[name])
    if not ok:
      raise ConversionError(f'{name} differs from the expected next state')

  camera = _vision_feed_tensors(converted, legacy_vision)
  glue = _policy_glue_tensors(converted)
  captured = _capture(converted, feed, list(camera.values()) + glue)

  dtypes = {i.name: _np_dtype(i) for i in legacy_vision.graph.input}
  vision = _run(legacy_vision, {x: captured[t].astype(dtypes[x]) for x, t in camera.items()})[OUTPUTS]
  for key in vision_keys:
    if key not in vision_slices:
      raise ConversionError(f'legacy vision output_slices has no {key}')
    d = _max_diff(got[OUTPUTS][..., slices[key]], vision[..., vision_slices[key]])
    if d is not None:
      raise ConversionError(f'vision slice {key} differs from the standalone legacy vision: max|d|={d}')

  policy = _run(legacy_policy, _policy_feed({t: captured[t] for t in glue}, legacy_policy))[OUTPUTS]
  for key, s in policy_slices.items():
    if key not in slices:
      raise ConversionError(f'converted output_slices has no policy slice {key}')
    d = _max_diff(got[OUTPUTS][..., slices[key]], policy[..., s])
    if d is not None:
      raise ConversionError(f'policy slice {key} differs from the standalone legacy policy: max|d|={d}')

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
    converted = swap_weights(template, legacy_vision, legacy_policy)
    print(f'model_checkpoint {_meta(converted, "model_checkpoint")}')
    if not args.no_verify:
      verify(template, converted, legacy_vision, legacy_policy)
      print(f'verify: vision equals the standalone legacy vision and, except {HIDDEN}, the template; next states '
            f'consistent; policy slices equal the standalone legacy policy; {RECURRENCE_STEPS}-step recurrence finite')
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
