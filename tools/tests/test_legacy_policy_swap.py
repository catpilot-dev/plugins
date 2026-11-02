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

HIDDEN = 64          # policy hidden width
BB, HS, Q = 64, 16, 4  # vision backbone width, hidden_state width, feature queue length
V = BB + 2 * HS      # vision output width: backbone feat + inner summarizer ('lead'), then the head ('hidden_state')
POLICY_SLICES = {'plan': slice(0, 990), 'desire_state': slice(990, 998), 'pad': slice(-2, None)}
VISION_SLICES = {'lead': slice(0, BB + HS), 'hidden_state': slice(BB + HS, V)}
TEMPLATE_SLICES = {**VISION_SLICES, 'plan': slice(V, V + 990), 'desire_state': slice(V + 990, V + 998), 'pad': slice(-2, None)}
GLUE = {'features_buffer': 'model/fb_full', 'desire_pulse': 'model/dp_out', 'traffic_convention': 'model/tc_out'}
VISION_FEED = {'img': '_unsafe_view', 'big_img': '_unsafe_view_1'}
POLICY_INPUTS = [('features_buffer', [1, 25, 512]), ('desire_pulse', [1, 25, 8]), ('traffic_convention', [1, 2])]
HEAD_W, HEAD_B = 'model/summarizer.0.weight', 'model/summarizer.0.bias'
INNER_W, INNER_B = 'model/vision_model.policy.summarizer.0.weight', 'model/vision_model.policy.summarizer.0.bias'


def _fp16(name, shape, rng, scale):
  return numpy_helper.from_array((rng.standard_normal(shape) * scale).astype(np.float16), name)


def _i64(name, values):
  return numpy_helper.from_array(np.array(values, np.int64), name)


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
  inits = [_i64(p('axes'), [1]),
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


def make_vision(seed, head_seed=None):
  """Backbone Conv, an inner summarizer Gemm feeding 'lead', and the top-level hidden_state head summarizer.0.
  head_seed None: the head is tied to the inner summarizer (weight copy, shared bias), like CD210.
  head_seed n: the head has its own retrained weight and bias, like POP."""
  rng = np.random.default_rng(1000 + seed)
  inner_w = _fp16('vision_model.policy.summarizer.0.weight', (HS, BB), rng, 0.05)
  inner_b = _fp16('vision_model.policy.summarizer.0.bias', (HS,), rng, 0.05)
  inits = [_fp16('conv_w', (BB, 24, 1, 1), rng, 0.001), inner_w, inner_b]
  if head_seed is None:
    head_w = numpy_helper.from_array(numpy_helper.to_array(inner_w), 'summarizer.0.weight')
    inits.append(head_w)
    head_b = inner_b.name
  else:
    hrng = np.random.default_rng(2000 + head_seed)
    inits += [_fp16('summarizer.0.weight', (HS, BB), hrng, 0.05), _fp16('summarizer.0.bias', (HS,), hrng, 0.05)]
    head_b = 'summarizer.0.bias'
  nodes = [helper.make_node('Cast', ['img'], ['_to_copy'], to=TensorProto.FLOAT16),
           helper.make_node('Cast', ['big_img'], ['_to_copy_1'], to=TensorProto.FLOAT16),
           helper.make_node('Concat', ['_to_copy', '_to_copy_1'], ['cat'], axis=1),
           helper.make_node('Conv', ['cat', 'conv_w'], ['conv']),
           helper.make_node('Flatten', ['conv'], ['feat'], axis=1),
           helper.make_node('Gemm', ['feat', inner_w.name, inner_b.name], ['inner'], transB=1),
           helper.make_node('Gemm', ['feat', 'summarizer.0.weight', head_b], ['hidden'], transB=1),
           helper.make_node('Concat', ['feat', 'inner', 'hidden'], ['outputs'], axis=1)]
  graph = helper.make_graph(nodes, 'vision',
                            [helper.make_tensor_value_info(n, TensorProto.UINT8, [1, 12, 1, 1]) for n in VISION_FEED],
                            [helper.make_tensor_value_info('outputs', TensorProto.FLOAT16, [1, V])], inits)
  return _model(graph, output_slices=_slices(VISION_SLICES), model_checkpoint=f'v{seed}/200')


def make_template(vision, policy):
  """Supercombo-shaped graph: vision tensors renamed model/*, policy node outputs prefixed model/p_, and glue that
  slices the camera frames out of new_img, casts the policy inputs, concatenates the halves and appends the
  hidden_state slice to the feature queue."""
  def vname(x):
    return VISION_FEED.get(x) or ('model/vision_outputs' if x == 'outputs' else x and 'model/' + x)

  def pname(x):
    return GLUE.get(x) or (x and 'model/p_' + x)

  def copy(n, rename):
    c = onnx.NodeProto()
    c.CopyFrom(n)
    c.input[:] = [rename(x) for x in n.input]
    c.output[:] = [rename(x) for x in n.output]
    return c

  def renamed(i, name):
    c = onnx.TensorProto()
    c.CopyFrom(i)
    c.name = name
    return c

  hs = TEMPLATE_SLICES['hidden_state']
  inits = [_i64('g/0', [0]), _i64('g/1', [1]), _i64('g/2', [2]), _i64('g/q', [Q]),
           _i64('g/hs_start', [hs.start]), _i64('g/hs_end', [hs.stop])]
  nodes = [helper.make_node('Slice', ['new_img', 'g/0', 'g/1', 'g/0'], ['_unsafe_view']),
           helper.make_node('Slice', ['new_img', 'g/1', 'g/2', 'g/0'], ['_unsafe_view_1'])]
  nodes += [helper.make_node('Cast', [src], [dst], to=TensorProto.FLOAT16) for src, dst in GLUE.items()]
  nodes += [copy(n, vname) for n in vision.graph.node] + [copy(n, pname) for n in policy.graph.node]
  nodes += [helper.make_node('Concat', ['model/vision_outputs', 'model/p_outputs'], ['model/all'], axis=1),
            helper.make_node('Cast', ['model/all'], ['outputs'], to=TensorProto.FLOAT),
            helper.make_node('Slice', ['outputs', 'g/hs_start', 'g/hs_end', 'g/1'], ['g/hidden']),
            helper.make_node('Unsqueeze', ['g/hidden', 'g/0'], ['g/hidden_q']),
            helper.make_node('Slice', ['state_feat_q', 'g/1', 'g/q', 'g/0'], ['g/feat_tail']),
            helper.make_node('Concat', ['g/feat_tail', 'g/hidden_q'], ['next_state_feat_q'], axis=0)]
  inits += [renamed(i, 'model/' + i.name) for i in vision.graph.initializer]
  inits += [renamed(i, 'model/p_' + i.name) for i in policy.graph.initializer]
  inputs = [helper.make_tensor_value_info('new_img', TensorProto.UINT8, [2, 12, 1, 1])]
  inputs += [helper.make_tensor_value_info(n, TensorProto.FLOAT, s) for n, s in POLICY_INPUTS]
  inputs.append(helper.make_tensor_value_info('state_feat_q', TensorProto.FLOAT, [Q, 1, HS]))
  outputs = [helper.make_tensor_value_info('outputs', TensorProto.FLOAT, [1, V + 1000]),
             helper.make_tensor_value_info('next_state_feat_q', TensorProto.FLOAT, [Q, 1, HS])]
  graph = helper.make_graph(nodes, 'supercombo', inputs, outputs, inits)
  return _model(graph, output_slices=_slices(TEMPLATE_SLICES),
                model_checkpoint=f'{_checkpoint(vision)}/{_checkpoint(policy)}')


@pytest.fixture
def template():
  return make_template(make_vision(0), make_policy(0))


@pytest.fixture
def retrained():
  return make_vision(0, head_seed=1)


@pytest.fixture
def legacy():
  return make_policy(1, prefix='legacy_')


def _init(m, name):
  return next(i for i in m.graph.initializer if i.name == name)


def _node(m, weight):
  return next(n for n in m.graph.node if len(n.input) > 1 and n.input[1] == weight)


def _negate(m, name):
  i = _init(m, name)
  i.raw_data = (-numpy_helper.to_array(i)).tobytes()


def test_weight_sequence_lists_fp16_weights_in_node_order():
  seq = lps.weight_sequence(make_policy(0))
  assert seq == [('Gemm', 'w1', (HIDDEN, 512)), ('Gemm', 'w2', (1000, HIDDEN + 10))]


def test_weight_sequence_dedupes_and_filters_policy_nodes(template):
  assert [n for _, n, _ in lps.weight_sequence(template)] == \
         ['model/conv_w', INNER_W, INNER_B, HEAD_W, 'model/p_w1', 'model/p_w2']  # shared INNER_B listed once
  assert [n for _, n, _ in lps.weight_sequence(template, lps.is_policy_node)] == ['model/p_w1', 'model/p_w2']


def test_vision_compatible_passes_for_same_weights(template):
  assert lps.check_vision_compatible(template, make_vision(0)) == \
         {'summarizer.0.weight': HEAD_W, 'vision_model.policy.summarizer.0.bias': INNER_B}


def test_vision_compatible_maps_a_retrained_summarizer(template, retrained):
  assert lps.check_vision_compatible(template, retrained) == \
         {'summarizer.0.weight': HEAD_W, 'summarizer.0.bias': INNER_B}


def test_vision_backbone_mismatch_names_both_checkpoints(template):
  with pytest.raises(lps.ConversionError) as e:
    lps.check_vision_compatible(template, make_vision(1, head_seed=1))
  assert 'v0/200' in str(e.value) and 'v1/200' in str(e.value)


def test_vision_summarizer_shape_mismatch_refuses(template, retrained):
  _init(retrained, 'summarizer.0.bias').CopyFrom(
    numpy_helper.from_array(np.zeros(HS + 1, np.float16), 'summarizer.0.bias'))
  with pytest.raises(lps.ConversionError, match='v0/200'):
    lps.check_vision_compatible(template, retrained)


def test_align_policy_maps_by_position(template, legacy):
  assert lps.align_policy(template, legacy) == {'model/p_w1': 'legacy_w1', 'model/p_w2': 'legacy_w2'}


def test_align_policy_refuses_shape_mismatch(template):
  with pytest.raises(lps.ConversionError, match='600, 512'):
    lps.align_policy(template, make_policy(1, hidden=600))


def test_swap_with_identical_vision_replaces_only_policy_arrays(template, legacy):
  before = {i.name: i.raw_data for i in template.graph.initializer}
  out = lps.swap_weights(template, make_vision(0), legacy)
  after = {i.name: i.raw_data for i in out.graph.initializer}
  assert after['model/p_w1'] == _init(legacy, 'legacy_w1').raw_data != before['model/p_w1']
  assert after['model/p_w2'] == _init(legacy, 'legacy_w2').raw_data != before['model/p_w2']
  assert {n: b for n, b in after.items() if n not in ('model/p_w1', 'model/p_w2')} == \
         {n: b for n, b in before.items() if n not in ('model/p_w1', 'model/p_w2')}
  assert list(out.graph.node) == list(template.graph.node)
  assert {i.name: i.raw_data for i in template.graph.initializer} == before


def test_swap_transplants_summarizer_and_unties_shared_bias(template, retrained, legacy):
  before = {i.name: i.raw_data for i in template.graph.initializer}
  out = lps.swap_weights(template, retrained, legacy)
  after = {i.name: i.raw_data for i in out.graph.initializer}
  assert after[HEAD_W] == _init(retrained, 'summarizer.0.weight').raw_data != before[HEAD_W]
  assert list(_node(out, HEAD_W).input) == ['model/feat', HEAD_W, HEAD_B]
  assert after[HEAD_B] == _init(retrained, 'summarizer.0.bias').raw_data
  assert list(_node(out, INNER_W).input) == ['model/feat', INNER_W, INNER_B]
  assert after[INNER_B] == before[INNER_B]
  assert set(after) == set(before) | {HEAD_B}
  changed = {HEAD_W, HEAD_B, 'model/p_w1', 'model/p_w2'}
  assert {n: b for n, b in after.items() if n not in changed} == {n: b for n, b in before.items() if n not in changed}
  head = _node(template, HEAD_W).output[0]
  assert [n for n in out.graph.node if n.output[0] != head] == [n for n in template.graph.node if n.output[0] != head]


def test_swap_sets_checkpoint_and_producer(template, retrained, legacy):
  out = lps.swap_weights(template, retrained, legacy)
  assert _checkpoint(out) == 'v0/200/p1/100'
  assert out.producer_name == 'catpilot-policy-swap'
  assert [(p.key, p.value) for p in out.metadata_props if p.key != 'model_checkpoint'] == \
         [(p.key, p.value) for p in template.metadata_props if p.key != 'model_checkpoint']


@pytest.mark.parametrize('head_seed', [None, 1])
def test_verify_passes_for_a_correct_swap(template, legacy, head_seed):
  vision = make_vision(0, head_seed=head_seed)
  lps.verify(template, lps.swap_weights(template, vision, legacy), vision, legacy)


def test_verify_fails_when_a_policy_array_is_wrong(template, retrained, legacy):
  out = lps.swap_weights(template, retrained, legacy)
  _negate(out, 'model/p_w2')
  with pytest.raises(lps.ConversionError, match='policy'):
    lps.verify(template, out, retrained, legacy)


def test_verify_fails_when_a_backbone_array_is_wrong(template, retrained, legacy):
  out = lps.swap_weights(template, retrained, legacy)
  _negate(out, 'model/conv_w')
  with pytest.raises(lps.ConversionError, match='vision slice lead'):
    lps.verify(template, out, retrained, legacy)


@pytest.mark.parametrize('name', [HEAD_W, HEAD_B])
def test_verify_fails_when_a_summarizer_array_is_corrupted(template, retrained, legacy, name):
  out = lps.swap_weights(template, retrained, legacy)
  _negate(out, name)
  with pytest.raises(lps.ConversionError, match='hidden_state'):
    lps.verify(template, out, retrained, legacy)


def test_verify_fails_when_the_feature_queue_is_wrong(template, retrained, legacy):
  out = lps.swap_weights(template, retrained, legacy)
  q = next(n for n in out.graph.node if 'next_state_feat_q' in n.output)
  q.input[:] = list(reversed(q.input))
  with pytest.raises(lps.ConversionError, match='next_state_feat_q'):
    lps.verify(template, out, retrained, legacy)


def _write_inputs(tmp_path, template, vision, policy):
  paths = [tmp_path / n for n in ('template.onnx', 'vision.onnx', 'policy.onnx')]
  for m, p in zip((template, vision, policy), paths):
    onnx.save(m, p)
  return [str(p) for p in paths]


def test_main_prints_catalog_files_object(tmp_path, capsys, template, retrained, legacy):
  out = tmp_path / 'out.onnx'
  assert lps.main(_write_inputs(tmp_path, template, retrained, legacy) + [str(out)]) == 0
  files = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
  data = out.read_bytes()
  assert files == {'name': 'driving_supercombo.onnx', 'url': '<fill in release asset URL>',
                   'sha256': hashlib.sha256(data).hexdigest(), 'size': len(data)}


def test_main_no_verify_withholds_the_catalog_object(tmp_path, capsys, template, retrained, legacy):
  out = tmp_path / 'out.onnx'
  assert lps.main(_write_inputs(tmp_path, template, retrained, legacy) + [str(out), '--no-verify']) == 0
  printed = capsys.readouterr()
  data = out.read_bytes()
  assert 'UNVERIFIED — do not publish' in printed.err
  lines = printed.out.splitlines()
  assert f'sha256 {hashlib.sha256(data).hexdigest()}' in lines and f'size {len(data)}' in lines
  assert lps.URL_PLACEHOLDER not in printed.out
  assert not any(line.lstrip().startswith('{') for line in lines)


def test_main_refuses_a_different_backbone_and_writes_nothing(tmp_path, capsys, template, legacy):
  out = tmp_path / 'out.onnx'
  assert lps.main(_write_inputs(tmp_path, template, make_vision(1, head_seed=1), legacy) + [str(out)]) == 1
  assert 'v1/200' in capsys.readouterr().err
  assert not out.exists()
