"""Tests for tools/legacy_policy_swap.py on tiny synthetic split models and a synthetic supercombo template."""
import base64
import hashlib
import json
import pathlib
import pickle
import sys

import pytest

onnx = pytest.importorskip('onnx')
if not hasattr(pytest.importorskip('onnxruntime'), 'InferenceSession'):  # a bare libonnxruntime.so dir imports as a namespace
  pytest.skip('onnxruntime Python bindings are not installed', allow_module_level=True)

import numpy as np  # noqa: E402
from onnx import TensorProto, helper, numpy_helper  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import legacy_policy_swap as lps  # noqa: E402

HIDDEN = 64
POLICY_SLICES = {'plan': slice(0, 990), 'desire_state': slice(990, 998), 'pad': slice(-2, None)}
TEMPLATE_SLICES = {'lead': slice(0, 1064), 'hidden_state': slice(1064, 1576),
                   'plan': slice(1576, 2566), 'desire_state': slice(2566, 2574), 'pad': slice(-2, None)}
GLUE = {'features_buffer': 'model/fb_full', 'desire_pulse': 'model/dp_out', 'traffic_convention': 'model/tc_out'}
POLICY_INPUTS = [('features_buffer', [1, 25, 512]), ('desire_pulse', [1, 25, 8]), ('traffic_convention', [1, 2])]


def _fp16(name, shape, rng, scale):
  return numpy_helper.from_array((rng.standard_normal(shape) * scale).astype(np.float16), name)


def _model(graph, **meta):
  m = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 20)], producer_name='pytorch')
  m.ir_version = 10
  helper.set_model_props(m, meta)
  return m


def _slices(slices):
  return base64.b64encode(pickle.dumps(slices)).decode()


def _checkpoint(m):
  return next(p.value for p in m.metadata_props if p.key == 'model_checkpoint')


def make_policy(seed, prefix='', hidden=HIDDEN):
  rng = np.random.default_rng(seed)
  p = lambda s: prefix + s  # noqa: E731
  inits = [numpy_helper.from_array(np.array([1], np.int64), p('axes')),
           _fp16(p('w1'), (hidden, 512), rng, 0.05),
           _fp16(p('w2'), (1000, hidden + 8 + 2), rng, 0.05)]
  nodes = [helper.make_node('ReduceMean', ['features_buffer', p('axes')], [p('feat')], keepdims=0),
           helper.make_node('ReduceMean', ['desire_pulse', p('axes')], [p('desire')], keepdims=0),
           helper.make_node('Gemm', [p('feat'), p('w1')], [p('hid')], transB=1),
           helper.make_node('Relu', [p('hid')], [p('act')]),
           helper.make_node('Concat', [p('act'), p('desire'), 'traffic_convention'], [p('cat')], axis=1),
           helper.make_node('Gemm', [p('cat'), p('w2')], ['outputs'], transB=1)]
  graph = helper.make_graph(nodes, 'policy',
                            [helper.make_tensor_value_info(n, TensorProto.FLOAT16, s) for n, s in POLICY_INPUTS],
                            [helper.make_tensor_value_info('outputs', TensorProto.FLOAT16, [1, 1000])], inits)
  return _model(graph, output_slices=_slices(POLICY_SLICES), model_checkpoint=f'p{seed}/100')


def make_vision(seed):
  rng = np.random.default_rng(1000 + seed)
  nodes = [helper.make_node('Cast', ['img'], ['img_h'], to=TensorProto.FLOAT16),
           helper.make_node('Cast', ['big_img'], ['big_img_h'], to=TensorProto.FLOAT16),
           helper.make_node('Concat', ['img_h', 'big_img_h'], ['x'], axis=1),
           helper.make_node('Conv', ['x', 'conv_w'], ['y']),
           helper.make_node('Flatten', ['y'], ['outputs'], axis=1)]
  graph = helper.make_graph(nodes, 'vision',
                            [helper.make_tensor_value_info(n, TensorProto.UINT8, [1, 12, 1, 1]) for n in ('img', 'big_img')],
                            [helper.make_tensor_value_info('outputs', TensorProto.FLOAT16, [1, 1576])],
                            [_fp16('conv_w', (1576, 24, 1, 1), rng, 0.001)])
  return _model(graph, model_checkpoint=f'v{seed}/200')


def make_template(vision, policy):
  """Supercombo-shaped graph: plain vision nodes, policy nodes whose outputs carry the model/p_ prefix, minimal glue."""
  def vname(x):
    return 'model/vision_outputs' if x == 'outputs' else x

  def pname(x):
    return GLUE.get(x) or (x and 'model/p_' + x)

  nodes = [helper.make_node('Cast', [src], [dst], to=TensorProto.FLOAT16) for src, dst in GLUE.items()]
  for n in vision.graph.node:
    c = onnx.NodeProto()
    c.CopyFrom(n)
    c.output[:] = [vname(x) for x in n.output]
    nodes.append(c)
  for n in policy.graph.node:
    c = onnx.NodeProto()
    c.CopyFrom(n)
    c.input[:] = [pname(x) for x in n.input]
    c.output[:] = [pname(x) for x in n.output]
    nodes.append(c)
  nodes += [helper.make_node('Concat', ['model/vision_outputs', 'model/p_outputs'], ['model/all'], axis=1),
            helper.make_node('Cast', ['model/all'], ['outputs'], to=TensorProto.FLOAT),
            helper.make_node('Identity', ['state_feat_q'], ['next_state_feat_q'])]
  inits = list(vision.graph.initializer)
  for i in policy.graph.initializer:
    c = onnx.TensorProto()
    c.CopyFrom(i)
    c.name = 'model/p_' + i.name
    inits.append(c)
  inputs = [helper.make_tensor_value_info(n, TensorProto.UINT8, [1, 12, 1, 1]) for n in ('img', 'big_img')]
  inputs += [helper.make_tensor_value_info(n, TensorProto.FLOAT, s) for n, s in POLICY_INPUTS]
  inputs.append(helper.make_tensor_value_info('state_feat_q', TensorProto.FLOAT, [4, 1, 16]))
  outputs = [helper.make_tensor_value_info('outputs', TensorProto.FLOAT, [1, 2576]),
             helper.make_tensor_value_info('next_state_feat_q', TensorProto.FLOAT, [4, 1, 16])]
  graph = helper.make_graph(nodes, 'supercombo', inputs, outputs, inits)
  return _model(graph, output_slices=_slices(TEMPLATE_SLICES),
                model_checkpoint=f'{_checkpoint(vision)}/{_checkpoint(policy)}')


@pytest.fixture
def template():
  return make_template(make_vision(0), make_policy(0))


@pytest.fixture
def legacy():
  return make_policy(1, prefix='legacy_')


def _init(m, name):
  return next(i for i in m.graph.initializer if i.name == name)


def _negate(m, name):
  i = _init(m, name)
  i.raw_data = (-numpy_helper.to_array(i)).tobytes()


def test_weight_sequence_lists_fp16_weights_in_node_order():
  seq = lps.weight_sequence(make_policy(0))
  assert seq == [('Gemm', 'w1', (HIDDEN, 512)), ('Gemm', 'w2', (1000, HIDDEN + 10))]


def test_weight_sequence_filter_keeps_only_policy_nodes(template):
  assert [op for op, _, _ in lps.weight_sequence(template)] == ['Conv', 'Gemm', 'Gemm']
  assert [n for _, n, _ in lps.weight_sequence(template, lps.is_policy_node)] == ['model/p_w1', 'model/p_w2']


def test_vision_identical_passes_for_same_weights(template):
  lps.check_vision_identical(template, make_vision(0))


def test_vision_mismatch_names_both_checkpoints(template):
  with pytest.raises(lps.ConversionError) as e:
    lps.check_vision_identical(template, make_vision(1))
  assert 'v0/200' in str(e.value) and 'v1/200' in str(e.value)


def test_align_policy_maps_by_position(template, legacy):
  assert lps.align_policy(template, legacy) == {'model/p_w1': 'legacy_w1', 'model/p_w2': 'legacy_w2'}


def test_align_policy_refuses_shape_mismatch(template):
  with pytest.raises(lps.ConversionError, match='600, 512'):
    lps.align_policy(template, make_policy(1, hidden=600))


def test_swap_replaces_only_policy_arrays(template, legacy):
  before = {i.name: i.raw_data for i in template.graph.initializer}
  out = lps.swap_policy(template, make_vision(0), legacy)
  after = {i.name: i.raw_data for i in out.graph.initializer}
  assert after['model/p_w1'] == _init(legacy, 'legacy_w1').raw_data != before['model/p_w1']
  assert after['model/p_w2'] == _init(legacy, 'legacy_w2').raw_data != before['model/p_w2']
  assert {n: b for n, b in after.items() if n not in ('model/p_w1', 'model/p_w2')} == \
         {n: b for n, b in before.items() if n not in ('model/p_w1', 'model/p_w2')}
  assert {i.name: i.raw_data for i in template.graph.initializer} == before


def test_swap_sets_checkpoint_and_producer(template, legacy):
  out = lps.swap_policy(template, make_vision(0), legacy)
  assert _checkpoint(out) == 'v0/200/p1/100'
  assert out.producer_name == 'catpilot-policy-swap'
  assert [(p.key, p.value) for p in out.metadata_props if p.key != 'model_checkpoint'] == \
         [(p.key, p.value) for p in template.metadata_props if p.key != 'model_checkpoint']


def test_verify_passes_for_a_correct_swap(template, legacy):
  lps.verify(template, lps.swap_policy(template, make_vision(0), legacy), legacy)


def test_verify_fails_when_a_policy_array_is_wrong(template, legacy):
  out = lps.swap_policy(template, make_vision(0), legacy)
  _negate(out, 'model/p_w2')
  with pytest.raises(lps.ConversionError, match='policy'):
    lps.verify(template, out, legacy)


def test_verify_fails_when_a_vision_array_is_wrong(template, legacy):
  out = lps.swap_policy(template, make_vision(0), legacy)
  _negate(out, 'conv_w')
  with pytest.raises(lps.ConversionError, match='vision'):
    lps.verify(template, out, legacy)


def test_verify_fails_when_a_state_output_differs(template, legacy):
  out = lps.swap_policy(template, make_vision(0), legacy)
  next(n for n in out.graph.node if 'next_state_feat_q' in n.output).op_type = 'Neg'
  with pytest.raises(lps.ConversionError, match='next_state_feat_q'):
    lps.verify(template, out, legacy)


def _write_inputs(tmp_path, template, vision, policy):
  paths = [tmp_path / n for n in ('template.onnx', 'vision.onnx', 'policy.onnx')]
  for m, p in zip((template, vision, policy), paths):
    onnx.save(m, p)
  return [str(p) for p in paths]


def test_main_prints_catalog_files_object(tmp_path, capsys, template, legacy):
  out = tmp_path / 'out.onnx'
  assert lps.main(_write_inputs(tmp_path, template, make_vision(0), legacy) + [str(out)]) == 0
  files = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
  data = out.read_bytes()
  assert files == {'name': 'driving_supercombo.onnx', 'url': '<fill in release asset URL>',
                   'sha256': hashlib.sha256(data).hexdigest(), 'size': len(data)}


def test_main_refuses_a_different_vision_and_writes_nothing(tmp_path, capsys, template, legacy):
  out = tmp_path / 'out.onnx'
  assert lps.main(_write_inputs(tmp_path, template, make_vision(1), legacy) + [str(out)]) == 1
  assert 'v1/200' in capsys.readouterr().err
  assert not out.exists()
