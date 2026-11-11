"""Road info overlay: which text goes to which font, the per-name font cache, and what gets drawn."""
import importlib.util
import os
import sys
import types

import pytest

_MOD_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'road_info_overlay.py')


@pytest.fixture
def overlay():
  """A fresh module per test: it keeps loaded fonts and the last road in module state."""
  spec = importlib.util.spec_from_file_location('road_info_overlay_under_test', _MOD_PATH)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


class TestLabel:
  def test_ref_and_name(self, overlay):
    assert overlay._label('S20', '外环高速') == 'S20 外环高速'

  def test_name_without_ref(self, overlay):
    assert overlay._label('', '人民大道') == '人民大道'

  def test_ref_without_name(self, overlay):
    assert overlay._label('S20', '') == 'S20'

  def test_nothing(self, overlay):
    assert overlay._label('', '') == ''

  def test_blank_name_is_no_name(self, overlay):
    assert overlay._label('S20', '  ') == 'S20'

  def test_long_name_is_cut_with_an_ellipsis(self, overlay):
    name = '沪' * (overlay.MAX_NAME_CHARS + 5)
    assert overlay._label('S4', name) == 'S4 ' + '沪' * (overlay.MAX_NAME_CHARS - 1) + '…'

  def test_long_ascii_name_is_cut_with_dots_the_ui_font_has(self, overlay):
    # without the system font an ASCII label is drawn by the UI font, which has no ellipsis
    name = 'x' * (overlay.MAX_NAME_CHARS + 5)
    assert overlay._label('', name) == 'x' * (overlay.MAX_NAME_CHARS - 3) + '...'

  def test_name_at_the_limit_is_kept_whole(self, overlay):
    name = '路' * overlay.MAX_NAME_CHARS
    assert overlay._label('', name) == name


class FakeRl:
  """Records what the overlay asks raylib for. Behaviour copied from the real build:
  a font that could not be loaded has texture id 0, and glyphs the file lacks are
  dropped from the loaded font (glyphCount < codepoints requested)."""

  class TextureFilter:
    TEXTURE_FILTER_TRILINEAR = 'trilinear'

  class _Ffi:
    def new(self, ctype, values):
      return list(values)

    def cast(self, ctype, buf):
      return buf

  def __init__(self, patched=True):
    self.ffi = self._Ffi()
    self.loads = []         # (path, size, codepoints)
    self.unloaded = []
    self.draws = []         # (how, font, text, x, y, size)
    self.rects = []
    self.load_fails = False
    self.absent_codepoints = set()   # glyphs the font file does not carry
    self._next_texture = 10
    if patched:   # gui_app wraps draw_text_ex and keeps the original here
      self._orig_draw_text_ex = lambda font, text, pos, size, spacing, tint: \
        self.draws.append(('orig', font, text, pos.x, pos.y, size))

  def load_font_ex(self, path, size, codepoints, count):
    codepoints = list(codepoints)[:count]
    self.loads.append((path, size, codepoints))
    if self.load_fails:
      return types.SimpleNamespace(texture=types.SimpleNamespace(id=0), glyphCount=0)
    self._next_texture += 1
    found = [cp for cp in codepoints if cp not in self.absent_codepoints]
    return types.SimpleNamespace(texture=types.SimpleNamespace(id=self._next_texture), glyphCount=len(found))

  def gen_texture_mipmaps(self, texture):
    pass

  def set_texture_filter(self, texture, mode):
    pass

  def unload_font(self, font):
    self.unloaded.append(font)

  def measure_text_ex(self, font, text, size, spacing):
    return types.SimpleNamespace(x=len(text) * size, y=size)

  def draw_text_ex(self, font, text, pos, size, spacing, tint):
    self.draws.append(('patched', font, text, pos.x, pos.y, size))

  def draw_rectangle_rounded(self, rect, roundness, segments, color):
    self.rects.append(rect)

  @staticmethod
  def Rectangle(x, y, width, height):
    return types.SimpleNamespace(x=x, y=y, width=width, height=height)

  @staticmethod
  def Vector2(x, y):
    return types.SimpleNamespace(x=x, y=y)

  @staticmethod
  def Color(r, g, b, a):
    return (r, g, b, a)


@pytest.fixture
def system_font(overlay, tmp_path):
  """Point the overlay at a font file that exists."""
  path = tmp_path / 'NotoSansSC-Regular.otf'
  path.write_bytes(b'not really a font')
  overlay.SYSTEM_FONT = str(path)
  return str(path)


class TestLabelFont:
  def test_loads_the_labels_glyphs_from_the_system_font(self, overlay, system_font):
    rl = FakeRl()
    font = overlay._label_font(rl, 'S20 外环高速')
    assert font is not None
    assert rl.loads == [(system_font, overlay.ATLAS_SIZE, sorted({ord(c) for c in 'S20 外环高速'}))]

  def test_same_label_is_loaded_once(self, overlay, system_font):
    rl = FakeRl()
    first = overlay._label_font(rl, '中环路')
    assert overlay._label_font(rl, '中环路') is first
    assert len(rl.loads) == 1

  def test_keeps_only_recent_labels(self, overlay, system_font):
    rl = FakeRl()
    labels = [f'路{i}' for i in range(overlay.MAX_LABEL_FONTS + 1)]
    fonts = [overlay._label_font(rl, t) for t in labels]
    assert rl.unloaded == [fonts[0]]
    assert overlay._label_font(rl, labels[-1]) is fonts[-1]   # still cached
    assert len(rl.loads) == len(labels)

  def test_a_label_used_again_is_not_the_one_dropped(self, overlay, system_font):
    rl = FakeRl()
    labels = [f'路{i}' for i in range(overlay.MAX_LABEL_FONTS)]
    fonts = [overlay._label_font(rl, t) for t in labels]
    overlay._label_font(rl, labels[0])            # touch the oldest
    overlay._label_font(rl, '新路')
    assert rl.unloaded == [fonts[1]]

  def test_no_system_font_file(self, overlay, tmp_path):
    overlay.SYSTEM_FONT = str(tmp_path / 'absent.otf')
    rl = FakeRl()
    assert overlay._label_font(rl, 'S20 外环高速') is None
    assert rl.loads == []

  def test_unloadable_font_is_not_tried_again(self, overlay, system_font):
    rl = FakeRl()
    rl.load_fails = True
    assert overlay._label_font(rl, 'S20 外环高速') is None
    assert overlay._label_font(rl, '中环路') is None
    assert len(rl.loads) == 1
    assert rl.unloaded == []

  def test_label_with_a_glyph_the_font_lacks(self, overlay, system_font):
    rl = FakeRl()
    rl.absent_codepoints = {ord('𰻝')}
    assert overlay._label_font(rl, '𰻝𰻝面路') is None
    assert len(rl.unloaded) == 1                       # the partial font is released
    assert overlay._label_font(rl, '𰻝𰻝面路') is None   # and the answer is remembered
    assert len(rl.loads) == 1
    assert overlay._label_font(rl, '中环路') is not None  # other labels are unaffected


@pytest.fixture
def stage(overlay, monkeypatch):
  """The overlay switched on, drawing into a 2000x1000 HUD through a fake raylib and UI font.

  The fake raylib measures text at one font size per character, one font size
  high; the UI font's helper at 10 px per character. FONT_SCALE is 1.25.
  """
  ui_font = object()
  state = types.SimpleNamespace(rl=FakeRl(), ui_font=ui_font, scale=1.25)
  app = types.SimpleNamespace(FontWeight=types.SimpleNamespace(SEMI_BOLD='semi-bold'), FONT_SCALE=state.scale)
  fonts = types.SimpleNamespace(
    get_font=lambda weight: state.ui_font,
    measure=lambda font, text, size: types.SimpleNamespace(x=len(text) * 10.0, y=float(size)))
  for name in ('openpilot', 'openpilot.system', 'openpilot.system.ui', 'openpilot.system.ui.lib'):
    monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.application', app)
  monkeypatch.setitem(sys.modules, 'fonts', fonts)
  overlay._is_enabled = lambda: True
  hud = FakeRl.Rectangle(0, 0, 2000, 1000)

  def draw(way_ref, road_name):
    monkeypatch.setitem(sys.modules, 'pyray', state.rl)
    overlay._read_road_info = lambda: (way_ref, road_name)
    overlay.on_render_overlay(None, hud)
    return state.rl

  state.draw = draw
  state.system_size = overlay.FONT_SIZE * state.scale * overlay.SYSTEM_FONT_SCALE
  return state


class TestRenderWithSystemFont:
  """Ref and name are one string in one font: the system font, which has both."""

  def test_ref_and_name_are_one_text_in_the_system_font(self, overlay, stage, system_font):
    rl = stage.draw('S20', '外环高速')
    ((_, font, text, _, _, _),) = rl.draws
    assert text == 'S20 外环高速'
    assert font is overlay._label_fonts['S20 外环高速']

  def test_drawn_unpatched_at_the_scaled_size(self, overlay, stage, system_font):
    # gui_app's draw_text_ex swaps in its own font when the UI language is Chinese,
    # and scales the size itself; the label must go around it, scaled here
    rl = stage.draw('S20', '外环高速')
    how, _, _, _, _, size = rl.draws[0]
    assert how == 'orig'
    assert size == pytest.approx(stage.system_size)

  def test_sized_to_the_ui_fonts_em(self, overlay):
    # raylib sizes a font by line height: the system font's em is 0.676 of it, Inter's 0.826
    assert overlay.SYSTEM_FONT_SCALE == pytest.approx(0.826 / 0.676, abs=0.03)

  def test_centred_on_one_background_above_the_bottom_margin(self, overlay, stage, system_font):
    rl = stage.draw('S20', '外环高速')
    (rect,) = rl.rects
    _, _, text, x, y, size = rl.draws[0]
    width = len(text) * size
    assert x + width / 2 == pytest.approx(1000)
    assert rect.x < x and rect.x + rect.width > x + width
    assert rect.y < y and rect.y + rect.height > y + size
    assert y + size == pytest.approx(1000 - overlay.BOTTOM_MARGIN)

  def test_ref_alone_uses_the_same_font(self, overlay, stage, system_font):
    rl = stage.draw('S20', '')
    ((how, font, text, _, _, size),) = rl.draws
    assert (how, text) == ('orig', 'S20')
    assert font is overlay._label_fonts['S20']
    assert size == pytest.approx(stage.system_size)

  def test_name_alone_when_the_road_has_no_ref(self, overlay, stage, system_font):
    rl = stage.draw('', '人民大道')
    assert [(how, text) for how, _, text, *_ in rl.draws] == [('orig', '人民大道')]
    assert len(rl.rects) == 1

  def test_unpatched_raylib_draws_at_the_same_size(self, overlay, stage, system_font):
    stage.rl = FakeRl(patched=False)
    rl = stage.draw('S20', '外环高速')
    assert [(text, size) for _, _, text, _, _, size in rl.draws] == [('S20 外环高速', pytest.approx(stage.system_size))]

  def test_does_not_wait_for_the_ui_font(self, overlay, stage, system_font):
    stage.ui_font = None
    rl = stage.draw('S20', '外环高速')
    assert len(rl.draws) == 1

  def test_no_road_draws_nothing(self, overlay, stage, system_font):
    rl = stage.draw('', '')
    assert rl.draws == [] and rl.rects == [] and rl.loads == []

  def test_switched_off_draws_nothing(self, overlay, stage, system_font):
    overlay._is_enabled = lambda: False
    rl = stage.draw('S20', '外环高速')
    assert rl.draws == [] and rl.rects == [] and rl.loads == []


class TestRenderWithoutSystemFont:
  """No system font (or a glyph it lacks): the UI font draws what it can, which is ASCII."""

  @pytest.fixture(autouse=True)
  def no_system_font(self, overlay, tmp_path):
    overlay.SYSTEM_FONT = str(tmp_path / 'absent.otf')

  def test_chinese_name_is_left_out(self, overlay, stage):
    rl = stage.draw('S20', '外环高速')
    assert [(how, font, text, size) for how, font, text, _, _, size in rl.draws] == [
      ('patched', stage.ui_font, 'S20', overlay.FONT_SIZE)]
    assert len(rl.rects) == 1

  def test_ascii_name_is_kept(self, overlay, stage):
    rl = stage.draw('A1', 'Outer Ring')
    assert [(how, font, text) for how, font, text, *_ in rl.draws] == [('patched', stage.ui_font, 'A1 Outer Ring')]

  def test_chinese_name_without_a_ref_draws_nothing(self, overlay, stage):
    rl = stage.draw('', '外环高速')
    assert rl.draws == [] and rl.rects == []

  def test_ui_font_not_loaded_yet_draws_nothing(self, overlay, stage):
    stage.ui_font = None
    rl = stage.draw('S20', '外环高速')
    assert rl.draws == [] and rl.rects == []

  def test_ref_the_ui_font_cannot_draw_is_not_shown(self, overlay, stage):
    rl = stage.draw('沪A20', '外环高速')
    assert rl.draws == [] and rl.rects == []

  def test_label_centred_with_the_ui_fonts_own_measure(self, overlay, stage):
    rl = stage.draw('S20', '')
    _, _, _, x, y, _ = rl.draws[0]
    assert x + 30 / 2 == pytest.approx(1000)          # 3 characters at 10 px
    assert y + overlay.FONT_SIZE == pytest.approx(1000 - overlay.BOTTOM_MARGIN)


def test_a_glyph_missing_from_the_system_font_falls_back_to_the_ref(overlay, stage, system_font):
  stage.rl.absent_codepoints = {ord('𰻝')}
  rl = stage.draw('S20', '𰻝𰻝面路')
  assert [(how, font, text) for how, font, text, *_ in rl.draws] == [('patched', stage.ui_font, 'S20')]
