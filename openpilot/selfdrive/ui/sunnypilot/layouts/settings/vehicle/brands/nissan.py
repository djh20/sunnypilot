"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp
from opendbc.car.nissan.values import CAR


class NissanSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.has_stop_and_go = False

    self.stop_and_go_toggle = toggle_item_sp(tr("Stop and Go (Beta)"), "", param="NissanStopAndGo", callback=self._on_toggle_changed)

    self.items = [self.stop_and_go_toggle]

  def _on_toggle_changed(self, _):
    self.update_settings()

  def stop_and_go_disabled_msg(self):
    if not self.has_stop_and_go:
      return tr("This feature is currently not available on this platform.")
    elif not ui_state.is_offroad():
      return tr("Enable \"Always Offroad\" in Device panel, or turn vehicle off to toggle.")
    return ""

  def update_settings(self):
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      platform = bundle.get("platform")
      self.has_stop_and_go = CAR[platform] != CAR.NISSAN_ALTIMA
    elif ui_state.CP is not None:
      self.has_stop_and_go = ui_state.CP.carFingerprint != CAR.NISSAN_ALTIMA

    disabled_msg = self.stop_and_go_disabled_msg()
    description = tr("Experimental feature to enable auto-resume during stop-and-go for certain supported Nissan ProPILOT platforms.")

    self.stop_and_go_toggle.action_item.set_enabled(self.has_stop_and_go and ui_state.is_offroad())
    self.stop_and_go_toggle.set_description(f"<b>{disabled_msg}</b><br><br>{description}" if disabled_msg else description)
