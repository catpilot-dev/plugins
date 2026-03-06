"""Model selector UI — injected into Software panel via ui.software_settings_extend hook.

All UI imports are lazy (inside on_software_settings_extend) to avoid circular
imports when the hook is loaded during SoftwareLayout.__init__.
"""
import json
import math
import subprocess
import shutil
from pathlib import Path

from model_swapper import ModelSwapper, ModelType

PLUGINS_DIR = '/data/plugins'
PYTHON_BIN = '/usr/local/venv/bin/python'

_SWAPPERS = {
  'driving': ModelSwapper(ModelType.DRIVING),
  'dm': ModelSwapper(ModelType.DM),
}

MODEL_TYPE_LABELS = {
  'driving': 'Driving Model',
  'dm': 'Driver Monitoring',
}

# Result codes for ModelActionDialog
ACTION_NONE = -1
ACTION_DELETE = 0
ACTION_CANCEL = 1
ACTION_ACTIVATE = 2


def _strip_emoji(text):
  return ''.join(c for c in text if ord(c) < 0x10000 or c.isalnum()).strip()


def _read_active(model_type):
  swapper = _SWAPPERS[model_type]
  try:
    raw = swapper.active_model_file.read_text().strip()
    data = json.loads(raw)
    model_id = data.get('id', raw)
    name = _strip_emoji(data.get('name', model_id))
    date = ''
    try:
      with open(swapper.models_dir / model_id / 'model_info.json') as f:
        date = json.load(f).get('date', '')
    except Exception:
      pass
    display = f"{name} ({date})" if date else name
    return model_id, display
  except Exception:
    return '', 'unknown'


def _list_models(model_type):
  models = _SWAPPERS[model_type].list_models()
  return [{'id': m['id'], 'name': m.get('name', m['id']), 'date': m.get('date', '')}
          for m in models if m.get('has_onnx')]


def _display_label(model):
  if model['date']:
    return f"{model['name']} ({model['date']})"
  return model['name']


def _swap_model(model_type, model_id):
  _SWAPPERS[model_type].swap_model(model_id)


def _delete_model(model_type, model_id):
  model_dir = _SWAPPERS[model_type].models_dir / model_id
  if model_dir.is_dir():
    shutil.rmtree(model_dir)


def _find_script(name):
  d = Path(PLUGINS_DIR) / 'model_selector'
  s = d / name
  return s if s.exists() else None


def on_software_settings_extend(default, layout):
  """Hook: ui.software_settings_extend — inject model selector into Software panel."""
  from openpilot.common.swaglog import cloudlog
  cloudlog.info("model_selector: on_software_settings_extend called")
  # Lazy imports to avoid circular imports during SoftwareLayout.__init__
  import pyray as rl
  from openpilot.selfdrive.ui.ui_state import ui_state
  from openpilot.system.ui.lib.application import gui_app, FontWeight
  from openpilot.system.ui.widgets import Widget, DialogResult
  from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
  from openpilot.system.ui.widgets.list_view import button_item
  from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
  from openpilot.system.ui.widgets.button import Button, ButtonStyle
  from openpilot.system.ui.widgets.label import Label

  class ModelActionDialog(Widget):
    """3-button dialog: [Delete] (red) | [Cancel] | [Activate] (blue)."""

    def __init__(self, model_name, is_active=False):
      super().__init__()
      self._result = ACTION_NONE
      self._label = Label(model_name, 70, FontWeight.BOLD, text_color=rl.Color(201, 201, 201, 255))
      self._delete_btn = Button('Delete', lambda: self._set_result(ACTION_DELETE), button_style=ButtonStyle.DANGER)
      self._cancel_btn = Button('Cancel', lambda: self._set_result(ACTION_CANCEL))
      self._activate_btn = Button('Activate', lambda: self._set_result(ACTION_ACTIVATE), button_style=ButtonStyle.PRIMARY)
      self._activate_btn.set_enabled(not is_active)

    def _set_result(self, result):
      self._result = result

    def _render(self, rect):
      margin = 200
      dialog_rect = rl.Rectangle(margin, margin, gui_app.width - 2 * margin, gui_app.height - 2 * margin)
      rl.draw_rectangle_rec(dialog_rect, rl.Color(27, 27, 27, 255))

      inner_margin = 50
      content = rl.Rectangle(dialog_rect.x + inner_margin, dialog_rect.y + inner_margin,
                             dialog_rect.width - 2 * inner_margin, dialog_rect.height - 2 * inner_margin)

      self._label.render(rl.Rectangle(content.x, content.y, content.width, 80))

      btn_h = 160
      btn_spacing = 30
      btn_y = content.y + content.height - btn_h
      btn_w = (content.width - 2 * btn_spacing) / 3

      self._delete_btn.render(rl.Rectangle(content.x, btn_y, btn_w, btn_h))
      self._cancel_btn.render(rl.Rectangle(content.x + btn_w + btn_spacing, btn_y, btn_w, btn_h))
      self._activate_btn.render(rl.Rectangle(content.x + 2 * (btn_w + btn_spacing), btn_y, btn_w, btn_h))

      return self._result

  class ModelSelectorUI:
    """Manages model selector widgets and background processes."""

    def __init__(self):
      self._model_btns = {}
      self._model_cache = {}
      self._check_proc = None

      self.items = []
      for model_type, label in MODEL_TYPE_LABELS.items():
        active_id, active_name = _read_active(model_type)
        self._model_cache[model_type] = _list_models(model_type)

        btn = button_item(label, 'SELECT', callback=lambda mt=model_type: self._on_model_select(mt))
        btn.action_item.set_value(active_name)
        self._model_btns[model_type] = btn
        self.items.append(btn)

      self._new_models_btn = button_item('New Models', 'CHECK', callback=self._on_check_new_models)
      self._new_models_btn.action_item.set_value('check on github')
      _orig_hint = self._new_models_btn.action_item.get_width_hint
      self._new_models_btn.action_item.get_width_hint = lambda _o=_orig_hint: math.ceil(_o())
      self.items.append(self._new_models_btn)

    def show(self):
      for model_type in MODEL_TYPE_LABELS:
        self._model_cache[model_type] = _list_models(model_type)
        _, active_name = _read_active(model_type)
        if model_type in self._model_btns:
          self._model_btns[model_type].action_item.set_value(active_name)

    def update(self):
      if self._check_proc is not None and self._check_proc.poll() is not None:
        self._on_check_complete()

    def _set_status(self, status):
      self._new_models_btn.action_item.set_value(status)

    def _on_model_select(self, model_type):
      models = self._model_cache.get(model_type, [])
      if not models:
        return

      active_id = _read_active(model_type)[0]
      options = [_display_label(m) for m in models]
      current = ''
      for i, m in enumerate(models):
        if m['id'] == active_id:
          current = options[i]
          break

      dlg = MultiOptionDialog(MODEL_TYPE_LABELS[model_type], options, current=current)

      def on_select(result):
        if result != DialogResult.CONFIRM:
          return
        for i, m in enumerate(models):
          if options[i] == dlg.selection:
            is_active = m['id'] == active_id
            action_dlg = ModelActionDialog(_display_label(m), is_active=is_active)

            def on_action(r, mt=model_type, mid=m['id'], prev=active_id):
              if r == ACTION_DELETE:
                if mid == prev:
                  gui_app.set_modal_overlay(
                    ConfirmDialog('Cannot delete the active model.', 'OK', cancel_text=''),
                    callback=lambda _: None)
                  return
                _delete_model(mt, mid)
                self._model_cache[mt] = _list_models(mt)
              elif r == ACTION_ACTIVATE:
                try:
                  _swap_model(mt, mid)
                except Exception:
                  gui_app.set_modal_overlay(
                    ConfirmDialog('Model swap failed.', 'OK', cancel_text=''),
                    callback=lambda _: None)
                  return

                _, new_name = _read_active(mt)
                if mt in self._model_btns:
                  self._model_btns[mt].action_item.set_value(new_name)

                reboot_dlg = ConfirmDialog('Model swapped. Reboot to activate.', 'Reboot', cancel_text='Cancel')

                def on_reboot(r2, mt2=mt, prev2=prev):
                  if r2 == DialogResult.CONFIRM:
                    ui_state.params.put_bool_nonblocking("DoReboot", True)
                  else:
                    try:
                      _swap_model(mt2, prev2)
                      _, reverted_name = _read_active(mt2)
                      if mt2 in self._model_btns:
                        self._model_btns[mt2].action_item.set_value(reverted_name)
                    except Exception:
                      pass

                gui_app.set_modal_overlay(reboot_dlg, callback=on_reboot)

            gui_app.set_modal_overlay(action_dlg, callback=on_action)
            return

      gui_app.set_modal_overlay(dlg, callback=on_select)

    def _on_check_new_models(self):
      script = _find_script('model_download.py')
      if not script:
        return

      if self._check_proc is not None and self._check_proc.poll() is None:
        return

      self._set_status('checking')
      self._new_models_btn.action_item.set_enabled(False)

      self._check_proc = subprocess.Popen(
        ['bash', '-c', f'{PYTHON_BIN} {script} update-registry >/dev/null 2>&1; {PYTHON_BIN} {script} check-updates'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
      )

    def _on_check_complete(self):
      proc = self._check_proc
      self._check_proc = None
      self._new_models_btn.action_item.set_enabled(True)

      if proc.returncode != 0:
        self._set_status('check failed')
        return

      try:
        updates = json.loads(proc.stdout.read().decode())
      except Exception:
        self._set_status('check failed')
        return

      total = updates.get('total', 0)
      if total == 0:
        self._set_status('up to date')
        return

      TYPE_PREFIX = {'driving': 'Driving', 'dm': 'DM'}
      new_models = []
      for key in ('driving', 'dm'):
        for m in updates.get(key, []):
          name = m.get('name', m.get('id', ''))
          new_models.append({'id': m['id'], 'type': key, 'name': name, 'date': m.get('date', ''),
                             'prefix': TYPE_PREFIX[key]})
      new_models.sort(key=lambda m: m.get('date', ''), reverse=True)

      self._set_status(f'{total} available')

      options = [f"{m['prefix']}: {_display_label(m)}" for m in new_models]
      dlg = MultiOptionDialog('New Models', options, current='')

      def on_result(result):
        if result != DialogResult.CONFIRM:
          return
        for i, m in enumerate(new_models):
          if options[i] == dlg.selection:
            self._start_download(m['type'], m['id'])
            return

      gui_app.set_modal_overlay(dlg, callback=on_result)

    def _start_download(self, model_type, model_id):
      script = _find_script('model_download.py')
      if not script:
        return

      self._set_status('downloading')
      self._new_models_btn.action_item.set_enabled(False)

      self._check_proc = subprocess.Popen(
        [PYTHON_BIN, str(script), 'download', model_id, '--type', model_type],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
      )

      self._download_model_type = model_type
      _orig_complete = self._on_check_complete

      def on_download_complete():
        proc = self._check_proc
        self._check_proc = None
        self._on_check_complete = _orig_complete
        self._new_models_btn.action_item.set_enabled(True)

        if proc.returncode != 0:
          self._set_status('download failed')
          return

        self._model_cache[self._download_model_type] = _list_models(self._download_model_type)
        self._set_status('download complete')

      self._on_check_complete = on_download_complete

  manager = ModelSelectorUI()
  layout._plugin_items.extend(manager.items)
  layout._plugin_updaters.append(manager.update)
  layout._plugin_show_cbs.append(manager.show)
