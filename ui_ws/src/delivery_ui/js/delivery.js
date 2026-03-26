/* ==========================================================================
   delivery.js — Load Cell Delivery + AprilTag Docking UI
   Depends on: app.js (SERVER_URL, showToast)
   ========================================================================== */

var _deliveryPollTimer = null;
var _deliveryRunning   = false;

/* ---------- START DELIVERY NODE ---------- */

function startDelivery() {
  fetch(SERVER_URL + "/delivery/start", { method: "POST" })
    .then(r => r.json())
    .then(function(d) {
      if (d.status === "already_running") {
        showToast("⚠ Delivery node already running", "info");
      } else {
        showToast("✅ Delivery node started", "success");
        _deliveryRunning = true;
        _startDeliveryPoll();
        _setDeliveryButtons(true);
      }
    })
    .catch(() => showToast("⚠ Failed to start delivery node", "error"));
}

/* ---------- STOP DELIVERY NODE ---------- */

function stopDelivery() {
  fetch(SERVER_URL + "/delivery/stop", { method: "POST" })
    .then(r => r.json())
    .then(function() {
      showToast("⏹ Delivery node stopped", "info");
      _deliveryRunning = false;
      _stopDeliveryPoll();
      _setDeliveryButtons(false);
      _resetDeliveryUI();
    })
    .catch(() => showToast("⚠ Failed to stop delivery node", "error"));
}

/* ---------- MANUAL TRIGGER (test without load cell) ---------- */

function triggerDelivery() {
  fetch(SERVER_URL + "/delivery/trigger", { method: "POST" })
    .then(r => r.json())
    .then(function(d) {
      if (d.status === "not_running") {
        showToast("⚠ Start the delivery node first", "error");
      } else if (d.status === "not_idle") {
        showToast("⚠ Robot is already on a mission (" + d.state + ")", "info");
      } else {
        showToast("🚀 Delivery triggered manually", "success");
      }
    })
    .catch(() => showToast("⚠ Failed to trigger delivery", "error"));
}

/* ---------- STATUS POLLING ---------- */

function _startDeliveryPoll() {
  if (_deliveryPollTimer) return;
  _deliveryPollTimer = setInterval(_pollDeliveryStatus, 800);
}

function _stopDeliveryPoll() {
  if (_deliveryPollTimer) { clearInterval(_deliveryPollTimer); _deliveryPollTimer = null; }
}

function _pollDeliveryStatus() {
  fetch(SERVER_URL + "/delivery/status")
    .then(function(r) {
      if (r.status === 404) {
        /* Endpoint not deployed — stop polling silently */
        _stopDeliveryPoll();
        _resetDeliveryUI();
        return null;
      }
      if (r.status === 204) { _resetDeliveryUI(); return null; }
      return r.json();
    })
    .then(function(d) {
      if (!d) return;
      _updateDeliveryUI(d);
    })
    .catch(function() {});
}

/* ---------- UI UPDATE ---------- */

var _STATE_LABELS = {
  "IDLE":                 { label: "💤 Idle — waiting for load",          color: "#6b7280", station: null },
  "DOCK_LOAD":            { label: "🎯 Docking at loading station",        color: "#8b5cf6", station: "load" },
  "WAIT_BEFORE_NAV":      { label: "⏳ Loaded — moving to delivery soon",  color: "#f59e0b", station: null },
  "NAVIGATE_TO_DELIVERY": { label: "🚗 Navigating to delivery",            color: "#3b82f6", station: "delivery" },
  "DOCK_DELIVERY":        { label: "🎯 Docking at delivery station",        color: "#8b5cf6", station: "delivery" },
  "WAIT_FOR_UNLOAD":      { label: "📦 At delivery — please unload",       color: "#10b981", station: "delivery" },
  "RETURN_WAIT":          { label: "⏳ Unloaded — returning home soon",    color: "#f59e0b", station: null },
  "NAVIGATE_TO_LOADING":  { label: "🏠 Navigating to loading station",     color: "#3b82f6", station: "load" },
  "DOCK_LOAD_RETURN":     { label: "🎯 Docking back at loading station",   color: "#8b5cf6", station: "load" },
};

function _updateDeliveryUI(d) {
  var stateEl    = document.getElementById("deliveryState");
  var weightEl   = document.getElementById("deliveryWeight");
  var distEl     = document.getElementById("deliveryDist");
  var tagEl      = document.getElementById("deliveryTag");
  var barEl      = document.getElementById("deliveryWeightBar");

  if (!stateEl) return;

  /* State label */
  var info = _STATE_LABELS[d.state] || { label: d.state, color: "#6b7280" };
  stateEl.textContent  = info.label;
  stateEl.style.color  = info.color;

  /* Weight */
  var kg = (d.weight || 0).toFixed(2);
  var threshold = (d.threshold || 2.0);
  weightEl.textContent = kg + " kg";
  weightEl.style.color = d.weight >= threshold ? "#10b981" : "#9ca3af";

  /* Weight bar */
  var pct = Math.min(100, (d.weight / threshold) * 100);
  barEl.style.width      = pct + "%";
  barEl.style.background = d.weight >= threshold ? "#10b981" : "#3b82f6";

  /* Distance */
  if (d.distance > 0) {
    distEl.textContent = d.distance.toFixed(2) + " m remaining";
    distEl.style.display = "block";
  } else {
    distEl.style.display = "none";
  }

  /* AprilTag — label which station it belongs to */
  var info = _STATE_LABELS[d.state] || {};
  var stationLabel = info.station === "load" ? "Loading"
                   : info.station === "delivery" ? "Delivery" : "";
  if (d.tag_visible) {
    tagEl.textContent = "🎯 " + stationLabel + " Tag #" + d.tag_id
                      + "  dist=" + (d.tag_z||0).toFixed(2) + "m"
                      + "  lat=" + (d.tag_x||0).toFixed(3) + "m";
    tagEl.style.color = "#10b981";
  } else if (info.station) {
    tagEl.textContent = "👁 Searching for " + stationLabel + " Tag #"
                      + (info.station === "load" ? d.load_tag : d.delivery_tag);
    tagEl.style.color = "#f59e0b";
  } else {
    tagEl.textContent = "—";
    tagEl.style.color = "#6b7280";
  }
}

function _resetDeliveryUI() {
  var stateEl  = document.getElementById("deliveryState");
  var weightEl = document.getElementById("deliveryWeight");
  var distEl   = document.getElementById("deliveryDist");
  var tagEl    = document.getElementById("deliveryTag");
  var barEl    = document.getElementById("deliveryWeightBar");
  if (!stateEl) return;
  stateEl.textContent  = "Node not running";
  stateEl.style.color  = "#6b7280";
  weightEl.textContent = "— kg";
  distEl.style.display = "none";
  tagEl.textContent    = "—";
  barEl.style.width    = "0%";
}

function _setDeliveryButtons(running) {
  var startBtn   = document.getElementById("deliveryStartBtn");
  var stopBtn    = document.getElementById("deliveryStopBtn");
  var triggerBtn = document.getElementById("deliveryTriggerBtn");
  if (!startBtn) return;
  startBtn.disabled   =  running;
  stopBtn.disabled    = !running;
  triggerBtn.disabled = !running;
}

/* ---------- INIT ---------- */

/* Called by switchView('delivery') in index.html */
function initDeliveryView() {
  fetch(SERVER_URL + "/delivery/status")
    .then(function(r) {
      if (r.status === 404) return null;   /* endpoint not deployed yet */
      if (r.status !== 204) {
        _deliveryRunning = true;
        _setDeliveryButtons(true);
        _startDeliveryPoll();
      }
      return null;
    })
    .catch(function() {});
}
