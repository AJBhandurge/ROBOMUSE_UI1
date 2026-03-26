/* ==========================================================================
   battery_monitor.js — JK BMS battery monitoring
   Topic: /jkbms_node/battery_state  (sensor_msgs/BatteryState)
   Topic: /jkbms_node/temperature    (sensor_msgs/Temperature)
   Topbar: battery-val, battery-bar, temp-val, battery-time
   ========================================================================== */

var BATTERY_TOTAL_AH  = 40.0;   /* total pack capacity in Ah */
var _lastCurrentA     = 0;      /* latest discharge current (A, positive = draining) */
var _ahConsumedEst    = 0;      /* Ah consumed since page load (integration estimate) */
var _lastTimestamp    = null;   /* for Δt integration */

/* ── helpers ─────────────────────────────────────────────────────────── */
function _setEl(id, text, color) {
  var el = document.getElementById(id);
  if (!el) return;
  if (text  !== undefined) el.textContent = text;
  if (color !== undefined) el.style.color = color;
}
function _setBar(id, pct) {
  var el = document.getElementById(id);
  if (!el) return;
  el.style.width = Math.max(0, Math.min(100, pct)) + "%";
  el.style.background = pct < 20 ? "#ef4444" : pct < 50 ? "#f97316" : "#22c55e";
}

/* ── format time helper ─────────────────────────────────────────────── */
function _fmtTime(hours) {
  if (hours < 0.02)  return "< 1m";
  if (hours < 1.0)   return Math.round(hours * 60) + "m";
  if (hours < 10.0)  return hours.toFixed(1) + "h";
  return Math.round(hours) + "h";
}

function _timeColor(hours) {
  if (hours < 0.5)  return "#ef4444";
  if (hours < 1.5)  return "#f97316";
  return "#22c55e";
}

/* ── init subscription ──────────────────────────────────────────────── */
function _initBattery() {

  /* BatteryState */
  var battTopic = new ROSLIB.Topic({
    ros:         ros,
    name:        "/jkbms_node/battery_state",
    messageType: "sensor_msgs/BatteryState"
  });

  battTopic.subscribe(function (msg) {
    /* ── SOC ── */
    var raw = parseFloat(msg.percentage) || 0;
    var pct = raw <= 1.0 ? Math.round(raw * 100) : Math.round(raw);
    _setEl("battery-val", pct + "%",
           pct < 20 ? "#ef4444" : pct < 50 ? "#f97316" : "#22c55e");
    _setBar("battery-bar", pct);

    /* ── Current (positive in JK BMS = charging) ── */
    var cur = parseFloat(msg.current) || 0;
    _lastCurrentA = -cur;   /* positive when discharging */

    /* ── Integrate Ah consumed ── */
    var now = Date.now();
    if (_lastTimestamp !== null) {
      var dtH = (now - _lastTimestamp) / 3600000;   /* ms → hours */
      if (_lastCurrentA > 0) {
        _ahConsumedEst += _lastCurrentA * dtH;
      }
    }
    _lastTimestamp = now;

    /* ── Runtime estimate ──────────────────────────────────────────────
     * Primary: use msg.capacity (remaining Ah from BMS) if available.
     * Fallback: estimate from SOC × total capacity.
     * Formula: remaining_Ah ÷ discharge_current = hours left
     * Also shows consumed Ah alongside.
     */
    var ahRemaining;
    var capField = parseFloat(msg.capacity) || 0;
    if (capField > 0.5) {
      ahRemaining = capField;                          /* BMS-reported remaining */
    } else {
      ahRemaining = (pct / 100) * BATTERY_TOTAL_AH;   /* estimated from SOC */
    }

    var ahConsumed = BATTERY_TOTAL_AH - ahRemaining;  /* Ah used from full */

    if (_lastCurrentA > 0.2) {
      /* Discharging: show time remaining */
      var hoursLeft = ahRemaining / _lastCurrentA;
      var timeStr   = _fmtTime(hoursLeft);
      /* Show: "3.2h | ↓5.1A" */
      _setEl("battery-time",
             timeStr + " | ↓" + _lastCurrentA.toFixed(1) + "A",
             _timeColor(hoursLeft));
    } else if (cur > 0.1) {
      /* Charging */
      _setEl("battery-time", "CHG ↑" + cur.toFixed(1) + "A", "#22c55e");
    } else {
      /* Idle — show consumed Ah */
      _setEl("battery-time",
             ahConsumed.toFixed(1) + "Ah used",
             "#64748b");
    }

    /* ── Warnings ── */
    if (typeof showToast === "function" && typeof RobotConfig !== "undefined") {
      if (pct <= RobotConfig.battery.critical_percent) {
        showToast("🔴 Battery CRITICAL: " + pct + "%", "error");
      } else if (pct <= RobotConfig.battery.warn_percent) {
        showToast("⚠ Battery low: " + pct + "%", "error");
      }
    }
  });

  /* Temperature */
  var tempTopic = new ROSLIB.Topic({
    ros:         ros,
    name:        "/jkbms_node/temperature",
    messageType: "sensor_msgs/Temperature"
  });
  tempTopic.subscribe(function (msg) {
    var t = parseFloat(msg.temperature) || 0;
    _setEl("temp-val", t.toFixed(1) + "°C",
           t > 55 ? "#ef4444" : t > 45 ? "#f97316" : "#0ea5e9");
  });
}

/* Subscribe on load and reconnect */
_initBattery();
ros.on("connection", function () { _initBattery(); });