/**
 * CRJ900 idle-descent planner — JavaScript port of engine/planner.py
 * Primary performance: Ben Irvine, line technique, 2026-11-22.
 * Manual values are references and limits only.
 */
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.CRJPlanner = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const FT_PER_NM = 6076.11548556;
  const T0_K = 288.15;
  const L_K_PER_M = 0.0065;
  const TROP_M = 11000.0;
  const T_TROP_K = 216.65;
  const G0 = 9.80665;
  const R_AIR = 287.05287;
  const A0_KT = 661.4788;
  const G_OVER_R = G0 / R_AIR;
  const MIN_GS_KT = 80.0;
  const DEFAULT_STEP_FT = 500.0;
  const DEFAULT_DESCENT_MACH = 0.74;
  const DEFAULT_HIGH_SPEED_IAS = 290.0;
  const DEFAULT_DECEL_210_NM = 5.0;
  const DEFAULT_IDLE_PATH_ANGLE_DEG = null; // null = calibrated v3.0 idle model; a number forces a fixed angle
  const LEGACY_IDLE_PATH_ANGLE_DEG = 3.0;
  const DEFAULT_TOD_VS_CAP_FPM = 3300.0; // max idle VS (highest observed idle rate, 2026-11-23)
  const DESCENT_IAS_MIN_KIAS = 250.0;
  const DESCENT_IAS_MAX_KIAS = 320.0;
  // v3.0 high-speed idle model (Ben Irvine flights 2026-11-23). Must match engine/planner.py.
  const IDLE_MACH_REF_VS_FPM = { light: 3100.0, medium: 3000.0, heavy: 2900.0 };
  const IDLE_MACH_REF_MACH = 0.74;
  const IDLE_MACH_REF_ALT_FT = 35000.0;
  const IDLE_250_ANCHOR_ALT_FT = 10000.0;
  const DEFAULT_ENERGY_IDLE_BAND = 0.10;
  const ENERGY_IDLE = "IDLE";
  const ENERGY_POWER_ON = "POWER_ON";
  const ENERGY_ADD_DRAG = "ADD_DRAG";
  const ENERGY_STAY_HIGH = "STAY_HIGH";
  const ENERGY_LABELS = { IDLE: "Idle", POWER_ON: "Power on", ADD_DRAG: "Add drag", STAY_HIGH: "Stay high" };
  const CLASSES = ["light", "medium", "heavy"];

  const PERF = {
    source_label: "Ben Irvine, line technique, 2026-11-22",
    idle_250_fpm: { heavy: 1600, medium: 1700, light: 1800 },
    idle_210_fpm: { heavy: 1000, medium: 1100, light: 1200 },
    method_a_kt_per_nm: 10,
    method_b_fpm: { 12000: 1200, 13000: 1300, 14000: 1400 },
    tod_vs_cap_fpm: DEFAULT_TOD_VS_CAP_FPM,
    idle_path_angle_deg: DEFAULT_IDLE_PATH_ANGLE_DEG,
    descent_mach_default: DEFAULT_DESCENT_MACH,
    high_speed_ias_kias: DEFAULT_HIGH_SPEED_IAS,
    decel_210_distance_nm: DEFAULT_DECEL_210_NM,
    step_ft: DEFAULT_STEP_FT,
  };

  function isaTemperatureK(altFt, isaDevC) {
    const hM = altFt * 0.3048;
    const t = hM <= TROP_M ? T0_K - L_K_PER_M * hM : T_TROP_K;
    return t + (isaDevC || 0);
  }

  function isaPressureRatio(altFt) {
    const hM = Math.max(altFt, 0) * 0.3048;
    if (hM <= TROP_M) {
      const t = T0_K - L_K_PER_M * hM;
      return Math.pow(t / T0_K, G0 / (R_AIR * L_K_PER_M));
    }
    const deltaTrop = Math.pow(T_TROP_K / T0_K, G0 / (R_AIR * L_K_PER_M));
    return deltaTrop * Math.exp((-G_OVER_R / T_TROP_K) * (hM - TROP_M));
  }

  function casToMachTas(casKt, altFt, isaDevC) {
    let t = Math.max(isaTemperatureK(altFt, isaDevC || 0), 150);
    const delta = Math.max(isaPressureRatio(altFt), 1e-6);
    const qcOverP0 = Math.pow(1 + 0.2 * Math.pow(casKt / A0_KT, 2), 3.5) - 1;
    const mach = Math.sqrt(Math.max(0, 5 * (Math.pow(qcOverP0 / delta + 1, 1 / 3.5) - 1)));
    const tasKt = mach * A0_KT * Math.sqrt(t / T0_K);
    return [mach, tasKt];
  }

  function machToCasTas(mach, altFt, isaDevC) {
    let t = Math.max(isaTemperatureK(altFt, isaDevC || 0), 150);
    const delta = Math.max(isaPressureRatio(altFt), 1e-6);
    const tasKt = mach * A0_KT * Math.sqrt(t / T0_K);
    const qcOverP0 = delta * (Math.pow(1 + 0.2 * mach * mach, 3.5) - 1);
    const casKt = A0_KT * Math.sqrt(Math.max(0, 5 * (Math.pow(qcOverP0 + 1, 1 / 3.5) - 1)));
    return [casKt, tasKt];
  }

  function crossoverAltitudeFt(descentMach, iasKias, isaDevC, loFt, hiFt) {
    loFt = loFt == null ? 5000 : loFt;
    hiFt = hiFt == null ? 45000 : hiFt;
    isaDevC = isaDevC || 0;
    for (let i = 0; i < 40; i++) {
      const mid = 0.5 * (loFt + hiFt);
      const tasIas = casToMachTas(iasKias, mid, isaDevC)[1];
      const tasM = machToCasTas(descentMach, mid, isaDevC)[1];
      if (tasM > tasIas) loFt = mid;
      else hiFt = mid;
    }
    return 0.5 * (loFt + hiFt);
  }

  function speedState(altFt, opts) {
    const descentMach = opts.descent_mach;
    const highSpeedIas = opts.high_speed_ias;
    const iasHeld = opts.ias_held;
    const isaDevC = opts.isa_dev_c || 0;
    if (iasHeld != null) {
      const [mach, tas] = casToMachTas(iasHeld, altFt, isaDevC);
      return { ias_kias: iasHeld, mach, tas_kt: tas, regime: "ias" };
    }
    const [casFromMach, tasFromMach] = machToCasTas(descentMach, altFt, isaDevC);
    if (casFromMach >= highSpeedIas) {
      const [mach, tas] = casToMachTas(highSpeedIas, altFt, isaDevC);
      return { ias_kias: highSpeedIas, mach, tas_kt: tas, regime: "ias" };
    }
    return { ias_kias: casFromMach, mach: descentMach, tas_kt: tasFromMach, regime: "mach" };
  }

  function groundspeedKt(tasKt, windComponentKt) {
    return Math.max(MIN_GS_KT, tasKt + (windComponentKt || 0));
  }

  function angleFromVsGs(vsFpm, gsKt) {
    const gs = Math.max(gsKt, MIN_GS_KT);
    return Math.atan((vsFpm * 60) / (gs * FT_PER_NM));
  }

  function vsFromAngleGs(angleRad, gsKt) {
    return (Math.tan(angleRad) * gsKt * FT_PER_NM) / 60;
  }

  function interpolateRate(lowW, lowR, highW, highR, weight) {
    if (highW === lowW) return lowR;
    const w = Math.min(Math.max(weight, Math.min(lowW, highW)), Math.max(lowW, highW));
    const t = (w - lowW) / (highW - lowW);
    return lowR + t * (highR - lowR);
  }

  const WEIGHT_MIN_LB = 55000;
  const WEIGHT_MAX_LB = 75100; // CRJ900 max landing weight (Ben Irvine, 2026-11-23)
  const WEIGHT_LIGHT_BELOW_LB = 65000;
  const WEIGHT_MEDIUM_MAX_LB = 70000;
  const fmtLb = (x) => Math.round(x).toLocaleString("en-US");

  function weightClassForLb(landingWeightLb) {
    const w = +landingWeightLb;
    if (!(w >= WEIGHT_MIN_LB && w <= WEIGHT_MAX_LB)) {
      throw new Error("Landing weight must be " + fmtLb(WEIGHT_MIN_LB) + "–" + fmtLb(WEIGHT_MAX_LB) + " lb (max landing weight " + fmtLb(WEIGHT_MAX_LB) + "), got " + (Number.isFinite(w) ? fmtLb(w) : String(landingWeightLb)) + ".");
    }
    if (w < WEIGHT_LIGHT_BELOW_LB) return "light";
    if (w <= WEIGHT_MEDIUM_MAX_LB) return "medium";
    return "heavy";
  }

  function resolveRates(weightClass, landingWeightLb) {
    const flags = [];
    let used = "weight_class";
    if (landingWeightLb != null) {
      const cls = weightClassForLb(landingWeightLb);
      if (CLASSES.indexOf(weightClass) >= 0 && weightClass !== cls) {
        flags.push("Landing weight " + fmtLb(+landingWeightLb) + " lb is " + cls + "; it overrides the " + weightClass + " selection.");
      }
      weightClass = cls;
      used = "landing_weight_class";
    }
    if (CLASSES.indexOf(weightClass) < 0) {
      throw new Error("weight_class must be one of " + CLASSES.join(", "));
    }
    return {
      class: weightClass,
      idle_250_fpm: +PERF.idle_250_fpm[weightClass],
      idle_210_fpm: +PERF.idle_210_fpm[weightClass],
      used,
      flags,
    };
  }


  function constraintBand(c, windowTarget) {
    windowTarget = windowTarget || "bottom";
    if (!c) return [null, null, null];
    const typ = String(c.typ || c.type || "none").replace(/-/g, "_").replace(/ /g, "_").toLowerCase();
    if ((typ === "at" || typ === "at_or_below" || typ === "at_or_above") && c.altitude_ft != null) {
      const alt = +c.altitude_ft;
      if (typ === "at") return [alt, alt, alt];
      if (typ === "at_or_below") return [alt, null, alt];
      return [alt, alt, null];
    }
    if (typ === "window") {
      const lo = c.min_ft != null ? c.min_ft : c.altitude_ft;
      const hi = c.max_ft != null ? c.max_ft : c.altitude_ft;
      if (lo == null || hi == null) return [null, lo, hi];
      const target = windowTarget === "bottom" ? +lo : +hi;
      return [target, +lo, +hi];
    }
    return [null, null, null];
  }

  function altSteps(altTop, altBottom, stepFt) {
    if (altTop <= altBottom + 0.5) return [];
    const steps = [];
    let a = altTop;
    stepFt = Math.max(50, +stepFt);
    while (a - altBottom > 0.5) {
      const b = Math.max(altBottom, a - stepFt);
      steps.push([a, b]);
      a = b;
    }
    return steps;
  }

  function integrateConstantVs(altTop, altBottom, iasKias, vsFpm, opts) {
    opts = opts || {};
    const isaDevC = opts.isa_dev_c || 0;
    const wind = opts.wind_component_kt || 0;
    const stepFt = opts.step_ft || DEFAULT_STEP_FT;
    const label = opts.label || "";
    const vsTop = opts.vs_top;
    const vsBottom = opts.vs_bottom;
    const stepsOut = [];
    let dist = 0;
    if (vsFpm <= 0 && vsTop == null) {
      return { distance_nm: 0, steps: [], vs_start_fpm: vsFpm, vs_end_fpm: vsFpm };
    }
    const pairs = altSteps(altTop, altBottom, stepFt);
    const totalH = Math.max(altTop - altBottom, 1e-6);
    for (const [a, b] of pairs) {
      const mid = 0.5 * (a + b);
      const st = speedState(mid, {
        descent_mach: 0.74,
        high_speed_ias: iasKias,
        ias_held: iasKias,
        isa_dev_c: isaDevC,
      });
      const gs = groundspeedKt(st.tas_kt, wind);
      let vs;
      if (vsTop != null && vsBottom != null) {
        const frac = (altTop - mid) / totalH;
        vs = vsTop + frac * (vsBottom - vsTop);
      } else {
        vs = vsFpm;
      }
      vs = Math.max(vs, 100);
      const dnm = ((a - b) / vs) * (gs / 60);
      dist += dnm;
      stepsOut.push({
        alt_top_ft: round(a, 1),
        alt_bottom_ft: round(b, 1),
        altitude_ft: round(mid, 1),
        ias_kias: round(st.ias_kias, 1),
        mach: round(st.mach, 3),
        tas_kt: round(st.tas_kt, 1),
        gs_kt: round(gs, 1),
        vs_fpm: round(vs, 0),
        distance_nm: round(dnm, 3),
        cumulative_distance_nm: round(dist, 3),
        segment: label,
      });
    }
    return {
      distance_nm: dist,
      steps: stepsOut,
      vs_start_fpm: vsTop != null ? vsTop : vsFpm,
      vs_end_fpm: vsBottom != null ? vsBottom : vsFpm,
    };
  }

  /**
   * v3.0 high-speed idle model: air-mass idle angle by regime, IAS and weight class.
   * VS = TAS × tan(angle). Mach regime: class VS at M0.74/FL350 ISA. IAS regime:
   * angle linear in IAS between the 250 kt anchor (250 kt table at 10,000 ft) and
   * 290 kt (= the Mach angle), extrapolated to 320. See data/flight_obs_2026-11-23.md.
   */
  function makeIdleModel(weightClass, idle250Fpm, fixedAngleDeg) {
    if (CLASSES.indexOf(weightClass) < 0) throw new Error("weight_class must be one of " + CLASSES.join(", "));
    const refTas = machToCasTas(IDLE_MACH_REF_MACH, IDLE_MACH_REF_ALT_FT, 0)[1];
    const tas250 = casToMachTas(250, IDLE_250_ANCHOR_ALT_FT, 0)[1];
    const machRefVs = IDLE_MACH_REF_VS_FPM[weightClass];
    const machAngle = angleFromVsGs(machRefVs, refTas);
    const angle250 = angleFromVsGs(+idle250Fpm, tas250);
    const fixed = fixedAngleDeg == null ? null : +fixedAngleDeg;
    function iasAngleRad(ias) {
      const v = Math.min(Math.max(+ias, DESCENT_IAS_MIN_KIAS), DESCENT_IAS_MAX_KIAS);
      return angle250 + (machAngle - angle250) * (v - 250) / 40;
    }
    function angleRad(st) {
      if (fixed != null) return degToRad(fixed);
      if (st.regime === "mach") return machAngle;
      return iasAngleRad(st.ias_kias);
    }
    function vsFpm(st) {
      return vsFromAngleGs(angleRad(st), st.tas_kt);
    }
    function summary() {
      const r3 = (x) => round(radToDeg(x), 3);
      return {
        model: fixed != null ? "fixed_angle" : "calibrated_2026_11_23",
        weight_class: weightClass,
        mach_ref_vs_fpm: machRefVs,
        mach_ref: "M" + IDLE_MACH_REF_MACH.toFixed(2) + " FL" + (IDLE_MACH_REF_ALT_FT / 100).toFixed(0) + " ISA, TAS " + refTas.toFixed(1) + " kt",
        mach_angle_deg: r3(machAngle),
        angle_250_deg: r3(angle250),
        angle_270_deg: r3(iasAngleRad(270)),
        angle_290_deg: r3(iasAngleRad(290)),
        angle_320_deg: r3(iasAngleRad(320)),
        fixed_angle_deg: fixed,
      };
    }
    return { isIdleModel: true, weightClass, fixedAngleDeg: fixed, machRefVs, machAngle, angle250, iasAngleRad, angleRad, vsFpm, summary };
  }

  function idleVsAtGs(gsKt, idleAngleRad, vsCapFpm, st) {
    if (idleAngleRad && idleAngleRad.isIdleModel) {
      if (!st) throw new Error("IdleModel needs the speed state");
      return Math.min(+vsCapFpm, idleAngleRad.vsFpm(st));
    }
    return Math.min(+vsCapFpm, vsFromAngleGs(idleAngleRad, gsKt));
  }

  function integrateIdlePath(altTop, altBottom, idleAngleRad, opts) {
    const descentMach = opts.descent_mach;
    const highSpeedIas = opts.high_speed_ias;
    const isaDevC = opts.isa_dev_c || 0;
    const wind = opts.wind_component_kt || 0;
    const stepFt = opts.step_ft || DEFAULT_STEP_FT;
    const vsCap = opts.vs_cap_fpm != null ? opts.vs_cap_fpm : DEFAULT_TOD_VS_CAP_FPM;
    const label = opts.label || "high_speed_idle";
    const stepsOut = [];
    let dist = 0;
    const vsList = [];
    let capApplied = false;
    for (const [a, b] of altSteps(altTop, altBottom, stepFt)) {
      const mid = 0.5 * (a + b);
      const st = speedState(mid, { descent_mach: descentMach, high_speed_ias: highSpeedIas, isa_dev_c: isaDevC });
      const gs = groundspeedKt(st.tas_kt, wind);
      const uncapped = idleAngleRad && idleAngleRad.isIdleModel ? idleAngleRad.vsFpm(st) : vsFromAngleGs(idleAngleRad, gs);
      let vs = Math.min(+vsCap, uncapped);
      if (vs + 0.5 < uncapped) capApplied = true;
      vs = Math.max(vs, 100);
      const dnm = ((a - b) / vs) * (gs / 60);
      dist += dnm;
      vsList.push(vs);
      stepsOut.push({
        alt_top_ft: round(a, 1),
        alt_bottom_ft: round(b, 1),
        altitude_ft: round(mid, 1),
        ias_kias: round(st.ias_kias, 1),
        mach: round(st.mach, 3),
        tas_kt: round(st.tas_kt, 1),
        gs_kt: round(gs, 1),
        vs_fpm: round(vs, 0),
        uncapped_vs_fpm: round(uncapped, 0),
        distance_nm: round(dnm, 3),
        cumulative_distance_nm: round(dist, 3),
        segment: label,
        regime: st.regime,
        path_angle_deg: round(radToDeg(angleFromVsGs(vs, gs)), 2),
      });
    }
    const startVs = vsList.length ? vsList[0] : null;
    const startGs = stepsOut.length ? stepsOut[0].gs_kt : null;
    const startAngle = startVs != null && startGs != null ? radToDeg(angleFromVsGs(startVs, startGs)) : null;
    return {
      distance_nm: dist,
      steps: stepsOut,
      vs_start_fpm: startVs,
      vs_end_fpm: vsList.length ? vsList[vsList.length - 1] : null,
      angle_deg: idleAngleRad && idleAngleRad.isIdleModel ? idleAngleRad.summary().mach_angle_deg : radToDeg(idleAngleRad),
      start_angle_deg: startAngle,
      cap_applied: capApplied,
      vs_cap_fpm: vsCap,
    };
  }

  function methodALevelDistanceNm(fromKias, toKias, ktPerNm) {
    ktPerNm = ktPerNm == null ? 10 : ktPerNm;
    if (ktPerNm <= 0) return 0;
    return Math.max(0, (fromKias - toKias) / ktPerNm);
  }


  function idleVsForIas(iasKias, idle250Fpm, idle210Fpm) {
    if (iasKias >= 250) return +idle250Fpm;
    if (iasKias <= 210) return +idle210Fpm;
    const tfrac = (250 - iasKias) / 40;
    return idle250Fpm + tfrac * (idle210Fpm - idle250Fpm);
  }

  function gradualDecelDistanceNm(fromKias, toKias, baseNm) {
    baseNm = baseNm == null ? 5 : baseNm;
    return Math.max(2, +baseNm * Math.abs(fromKias - toKias) / 40);
  }

  function idleStartAltitude(endAltFt, endIasKias, distanceNm, opts) {
    if (distanceNm <= 0.05) return +endAltFt;
    const endAlt = +endAltFt;
    const high = +opts.highSpeedIas;
    const gate = +opts.gateAlt;
    const cruise = +opts.cruiseAlt;
    const method = opts.method;
    const idle250 = +opts.idle250;
    const idle210 = +opts.idle210;
    const idleAngle = opts.idleAngle;
    const vsCap = +opts.vsCap;
    const isaDevC = opts.isaDevC || 0;
    const wind = opts.wind || 0;
    const descentMach = opts.descentMach;
    const methodAKtNm = opts.methodAKtNm == null ? 10 : +opts.methodAKtNm;
    const decelBaseNm = opts.decelBaseNm == null ? 5 : +opts.decelBaseNm;
    const stepNm = opts.stepNm == null ? 0.25 : +opts.stepNm;
    const endIas = endIasKias != null ? +endIasKias : (endAlt <= gate + 1 ? 250 : high);
    let alt = endAlt;
    let remaining = +distanceNm;
    // v3.0: speed already held at the start fix (null = 250). See engine/planner.py.
    const top = opts.startIas == null ? 250 : Math.min(250, Math.max(+opts.startIas, Math.min(endIas, 250)));
    const decelNm = endIas < top - 0.5 ? gradualDecelDistanceNm(top, endIas, decelBaseNm) : 0;
    let decelLeft = decelNm;
    let ias = endIas;
    const dLevel = method === "A" ? methodALevelDistanceNm(high, 250, methodAKtNm) : 0;
    let levelLeft = dLevel;
    const bTop = method === "B" && opts.methodBVs ? gate + (+opts.methodBOffset || 0) : null;

    function gsAt(a, iasHeld) {
      const st = speedState(a, {
        descent_mach: descentMach,
        high_speed_ias: high,
        ias_held: iasHeld,
        isa_dev_c: isaDevC,
      });
      return [groundspeedKt(st.tas_kt, wind), st];
    }

    let safety = 0;
    while (remaining > 1e-6 && alt < cruise + 100 && safety < 20000) {
      safety++;
      let ds = Math.min(stepNm, remaining);
      let vs = null;
      let iasHold;
      if (decelLeft > 0 && ias < top - 0.5) {
        vs = idleVsForIas(ias, idle250, idle210);
        iasHold = ias;
        ds = Math.min(ds, decelLeft);
      } else if (alt < gate - 1) {
        ias = ias != null ? Math.max(ias, top) : top;
        vs = idleVsForIas(ias, idle250, idle210);
        iasHold = ias;
      } else if (method === "A" && levelLeft > 0 && alt <= gate + 80) {
        vs = 0;
        iasHold = 250;
        ds = Math.min(ds, levelLeft);
        alt = Math.max(alt, gate);
      } else if (method === "B" && bTop != null && alt < bTop - 1) {
        vs = +opts.methodBVs;
        iasHold = Math.max(ias != null ? ias : 250, 250);
      } else {
        vs = null;
        iasHold = null;
      }
      const [gs, gst] = gsAt(alt, iasHold);
      if (vs == null) vs = idleVsAtGs(gs, idleAngle, vsCap, gst);
      const dAlt = vs * (ds / Math.max(gs, MIN_GS_KT)) * 60;
      alt += dAlt;
      remaining -= ds;
      if (decelLeft > 0 && ias < top - 0.5) {
        const frac = ds / Math.max(decelNm, 1e-6);
        ias = Math.min(top, ias + (top - endIas) * frac);
        decelLeft -= ds;
        if (decelLeft <= 0) ias = top;
      } else if (method === "A" && vs === 0 && levelLeft > 0) {
        levelLeft -= ds;
        if (levelLeft <= 0) ias = high;
      } else if (method === "B" && bTop != null && alt >= bTop - 1) {
        ias = high;
      }
    }
    return Math.min(alt, cruise);
  }

  function constraintFloorCeiling(r) {
    const typ = (r.constraint_typ || "none").replace(/-/g, "_");
    const lo = r.constraint_min_ft;
    const hi = r.constraint_max_ft;
    const tgt = r.constraint_target_ft;
    if (typ === "at") {
      const a = tgt != null ? tgt : lo != null ? lo : hi;
      return [a, a];
    }
    if (typ === "window") return [lo, hi];
    if (typ === "at_or_below") return [null, hi != null ? hi : tgt];
    if (typ === "at_or_above") return [lo != null ? lo : tgt, null];
    return [null, null];
  }

  function idleDescentDistance(startAltFt, endAltFt, endIasKias, opts) {
    if (startAltFt <= endAltFt + 50) return 0;
    let lo = 0, hi = 250;
    for (let i = 0; i < 28; i++) {
      const mid = 0.5 * (lo + hi);
      const reached = idleStartAltitude(endAltFt, endIasKias, mid, opts);
      if (reached >= startAltFt - 15) hi = mid;
      else lo = mid;
    }
    return hi;
  }

  function roundAlt100Ft(altFt) {
    if (altFt == null) return null;
    return Math.round(+altFt / 100) * 100;
  }

  function roundToTenth(x) {
    // Half away from zero so -3.75 becomes -3.8, matching Python round for these offsets.
    const s = x < 0 ? -1 : 1;
    return s * Math.round(Math.abs(x) * 10) / 10;
  }

  function formatAlongTrackIdent(baseName, offsetNm) {
    const x = roundToTenth(+offsetNm);
    if (Math.abs(x) < 0.05) return baseName;
    if (Math.abs(x - Math.round(x)) < 0.05) {
      const n = Math.round(x);
      return n < 0 ? baseName + "/" + n : baseName + "/+" + n;
    }
    const s = x.toFixed(1);
    return x < 0 ? baseName + "/" + s : baseName + "/+" + s;
  }

  function publishedRestrictionText(r) {
    if (!r) return null;
    const typ = (r.constraint_typ || "none").replace(/-/g, "_");
    const lo = r.constraint_min_ft, hi = r.constraint_max_ft, tgt = r.constraint_target_ft;
    if (typ === "window" && lo != null && hi != null) return "window " + Math.round(lo) + "–" + Math.round(hi);
    if (typ === "at") {
      const a = tgt != null ? tgt : lo != null ? lo : hi;
      return a != null ? "at " + Math.round(a) : null;
    }
    if (typ === "at_or_below") {
      const a = hi != null ? hi : tgt;
      return a != null ? "at or below " + Math.round(a) : null;
    }
    if (typ === "at_or_above") {
      const a = lo != null ? lo : tgt;
      return a != null ? "at or above " + Math.round(a) : null;
    }
    return null;
  }

  function clipHardToPublished(hard, r) {
    const typ = (r.constraint_typ || "none").replace(/-/g, "_");
    const lo = r.constraint_min_ft, hi = r.constraint_max_ft, tgt = r.constraint_target_ft;
    if (typ === "at") {
      const a = tgt != null ? tgt : lo != null ? lo : hi;
      if (a == null) return [hard, true, null];
      if (Math.abs(hard - a) <= 50) return [Math.abs(a - roundAlt100Ft(a)) < 1 ? a : hard, true, null];
      return [Math.round(a), false, "Hard altitude " + hard + " ft is not the published 'at' " + Math.round(a) + " ft; entered " + Math.round(a) + " so it stays legal."];
    }
    let low = null, high = null;
    if (typ === "window") { low = lo; high = hi; }
    else if (typ === "at_or_below") high = hi != null ? hi : tgt;
    else if (typ === "at_or_above") low = lo != null ? lo : tgt;
    else return [hard, true, null];
    let entered = hard, flag = null;
    if (high != null && entered > high + 0.5) {
      entered = Math.abs(high - Math.round(high)) < 0.5 ? Math.round(high) : roundAlt100Ft(high);
      if (entered > high) entered = Math.round(high);
      flag = "Rounded path altitude " + hard + " ft is above the published maximum " + Math.round(high) + " ft; entered " + entered + " so it stays legal.";
    }
    if (low != null && entered < low - 0.5) {
      entered = Math.abs(low - Math.round(low)) < 0.5 ? Math.round(low) : roundAlt100Ft(low);
      if (entered < low) entered = Math.round(low);
      flag = "Rounded path altitude " + hard + " ft is below the published minimum " + Math.round(low) + " ft; entered " + entered + " so it stays legal.";
    }
    return [entered, flag == null, flag];
  }

  function buildFmsLegs(outputWps, opts) {
    const MERGE_NM = 1.0;
    const star = outputWps.filter((w) => !w.synthetic && !w.is_builtin && w.planned_alt_ft != null && w.distance_from_tod_nm != null)
      .slice()
      .sort((a, b) => a.distance_from_tod_nm - b.distance_from_tod_nm);
    function starByName(name) { return star.find((s) => s.name === name); }
    const idleOpts = {
      idleAngle: opts.idleAngle,
      vsCap: opts.vsCap,
      idle250: opts.idle250,
      idle210: opts.idle210,
      method: opts.method,
      gateAlt: opts.gateAlt != null ? opts.gateAlt : 10000,
      methodBOffset: opts.methodBOffset || 0,
      methodBVs: opts.methodBVs,
      methodAKtNm: opts.methodAKtNm != null ? opts.methodAKtNm : 10,
      decelBaseNm: opts.decelBaseNm != null ? opts.decelBaseNm : 5,
      descentMach: opts.descentMach,
      highSpeedIas: opts.highSpeedIas,
      isaDevC: opts.isaDevC,
      wind: opts.wind,
      cruiseAlt: opts.cruiseAlt,
    };
    const raw = [];
    function add(dist, alt, speed, role, note, source, publishedSrc) {
      if (dist == null || alt == null) return;
      raw.push({ dist_nm: +dist, path_alt_ft: +alt, speed_kias: speed == null ? null : +speed, role, note, source_wp: source || null, published_src: publishedSrc || null });
    }
    star.forEach((s) => {
      add(s.distance_from_tod_nm, s.planned_alt_ft, s.planned_ias_kias, "star_fix",
        "Crossing restriction at " + s.name + ". Hard altitude on the stay-high idle path.", s, s);
    });
    (opts.decelSegments || []).forEach((seg) => {
      const typ = seg.type || "";
      const fixName = seg.before_fix;
      const base = fixName ? starByName(fixName) : null;
      if (!base || base.distance_from_tod_nm == null) return;
      const dseg = +(seg.distance_nm || 0);
      if (dseg <= 0.15) return;
      const pos = +base.distance_from_tod_nm - dseg;
      if (typ === "method_A_level_off") {
        add(pos, seg.altitude_ft || seg.end_altitude_ft || opts.gateAlt, opts.highSpeedIas, "level_off_start",
          "Start the method A level-off, " + dseg.toFixed(1) + " NM before " + fixName + ", to be 250 KIAS at " + fixName + ".");
      } else if (typ === "method_B_shallow_descent") {
        add(pos, seg.start_altitude_ft, opts.highSpeedIas, "method_b_start",
          "Start the method B shallow descent, " + dseg.toFixed(1) + " NM before " + fixName + ".");
      } else if (typ.indexOf("gradual_decel") === 0 || typ.indexOf("decel") === 0) {
        add(pos, seg.start_altitude_ft, seg.from_kias, "decel_start",
          "Start slowing from " + (+seg.from_kias).toFixed(0) + " to " + (+seg.to_kias).toFixed(0) + " KIAS, " + dseg.toFixed(1) + " NM before " + fixName + ".");
      }
    });
    for (let i = 1; i < star.length; i++) {
      const prev = star[i - 1], nxt = star[i];
      const prevAlt = +prev.planned_alt_ft, nxtAlt = +nxt.planned_alt_ft;
      if (prevAlt <= nxtAlt + 50) continue;
      const nxtIas = nxt.planned_ias_kias;
      const dIdle = idleDescentDistance(prevAlt, nxtAlt, nxtIas, idleOpts);
      const leg = +nxt.distance_from_tod_nm - +prev.distance_from_tod_nm;
      if (dIdle < leg - 0.4) {
        const pos = +nxt.distance_from_tod_nm - dIdle;
        if (pos > +prev.distance_from_tod_nm + 0.4) {
          add(pos, prevAlt, prev.planned_ias_kias, "descent_start",
            "Start the late idle descent toward " + nxt.name + ", " + dIdle.toFixed(1) + " NM before " + nxt.name + ".");
        }
      }
      if (prevAlt > 10050 && nxtAlt < 9950) {
        const d10 = idleDescentDistance(10000, nxtAlt, nxtIas, idleOpts);
        const pos10 = +nxt.distance_from_tod_nm - d10;
        if (+prev.distance_from_tod_nm + 0.2 < pos10 && pos10 < +nxt.distance_from_tod_nm - 0.2) {
          add(pos10, 10000, 250, "gate_10000",
            "Built-in 250 KIAS at 10,000 ft, " + Math.max(0, +nxt.distance_from_tod_nm - pos10).toFixed(1) + " NM before " + nxt.name + ".");
        }
      }
    }
    raw.sort((a, b) => a.dist_nm - b.dist_nm || (a.role === "star_fix" ? -1 : 1) - (b.role === "star_fix" ? -1 : 1));
    const merged = [];
    raw.forEach((pt) => {
      if (merged.length && pt.dist_nm - merged[merged.length - 1].dist_nm < MERGE_NM) {
        const a = merged[merged.length - 1];
        const b = pt;
        const starOne = a.role === "star_fix" ? a : b.role === "star_fix" ? b : null;
        if (starOne) {
          const other = starOne === a ? b : a;
          starOne.note = starOne.note + " Combined with " + other.role.replace(/_/g, " ") + " (" + other.note + ").";
          if (starOne.speed_kias == null) starOne.speed_kias = other.speed_kias;
          starOne.merged = true;
          const nm = starOne.source_wp && starOne.source_wp.name ? starOne.source_wp.name : "this fix";
          starOne.merge_warning = "A " + other.role.replace(/_/g, " ") + " was within 1 NM and was merged into " + nm + ".";
          merged[merged.length - 1] = starOne;
        } else {
          const altDiff = Math.abs((a.path_alt_ft || 0) - (b.path_alt_ft || 0));
          a.note = a.note + " Merged with " + b.role.replace(/_/g, " ") + " (" + b.note + ").";
          a.merged = true;
          let warn = "Two generated points were " + (b.dist_nm - a.dist_nm).toFixed(1) + " NM apart (under 1 NM) and were merged.";
          if (altDiff > 100) warn += " Altitudes differed (" + roundAlt100Ft(a.path_alt_ft) + " vs " + roundAlt100Ft(b.path_alt_ft) + " ft); kept the earlier (higher) hard altitude so VNAV starts in time.";
          a.merge_warning = warn;
          if (a.speed_kias == null) a.speed_kias = b.speed_kias;
        }
        return;
      }
      pt.merged = false;
      merged.push(pt);
    });
    function identFor(pt) {
      if (pt.role === "star_fix" && pt.source_wp) return [pt.source_wp.name, null, null];
      const dist = pt.dist_nm;
      let prev = null, nxt = null;
      star.forEach((s) => {
        const sd = +s.distance_from_tod_nm;
        if (sd <= dist + 0.05) prev = s;
        if (sd >= dist - 0.05 && nxt == null) nxt = s;
      });
      if (prev && nxt && prev !== nxt) {
        const dBack = +nxt.distance_from_tod_nm - dist;
        const dFwd = dist - +prev.distance_from_tod_nm;
        if (dBack <= dFwd + 0.05) return [formatAlongTrackIdent(nxt.name, -dBack), nxt.name, -dBack];
        return [formatAlongTrackIdent(prev.name, dFwd), prev.name, dFwd];
      }
      if (nxt) {
        const off = dist - +nxt.distance_from_tod_nm;
        return [formatAlongTrackIdent(nxt.name, off), nxt.name, off];
      }
      if (prev) {
        const off = dist - +prev.distance_from_tod_nm;
        return [formatAlongTrackIdent(prev.name, off), prev.name, off];
      }
      return ["PBD", null, null];
    }
    const restricted = star.filter((x) => !x.open).map((x) => +x.distance_from_tod_nm);
    const openAfter = restricted.length ? Math.max(...restricted) : null;
    return merged.map((pt) => {
      const [ident, base, offset] = identFor(pt);
      let hard = roundAlt100Ft(pt.path_alt_ft);
      const pub = pt.published_src;
      const pubText = pub ? publishedRestrictionText(pub) : null;
      let inside = true, pubFlag = null;
      if (pub) {
        const clipped = clipHardToPublished(hard, pub);
        hard = clipped[0]; inside = clipped[1]; pubFlag = clipped[2];
      }
      const spdI = pt.speed_kias == null ? null : Math.round(pt.speed_kias);
      const isOpen = pt.role === "star_fix" ? !!(pt.source_wp && pt.source_wp.open)
        : openAfter != null && pt.dist_nm > openAfter + 0.05;
      let enter, note = pt.note;
      if (isOpen) {
        // Past the last altitude restriction: no invented hard altitude.
        hard = null; inside = true; pubFlag = null;
        note = note + " Open: no altitude restriction here, so no hard altitude. " +
          "Set one in the app if you want VNAV to target it.";
        enter = spdI != null ? spdI + " / —" : "—";
      } else enter = spdI != null ? spdI + " / " + hard : String(hard);
      return {
        ident,
        is_star_fix: pt.role === "star_fix",
        is_along_track: pt.role !== "star_fix",
        base_fix: pt.role !== "star_fix" ? base : (pt.source_wp && pt.source_wp.name),
        offset_nm: pt.role === "star_fix" || offset == null ? null : roundToTenth(offset),
        role: pt.role,
        hard_altitude_ft: hard,
        speed_kias: spdI,
        enter,
        enter_syntax: "generic",
        ident_syntax: pt.role !== "star_fix" ? "manual" : "star",
        published_restriction: pubText,
        published_ok: inside,
        published_flag: pubFlag,
        note,
        merged: !!pt.merged,
        merge_warning: pt.merge_warning || null,
        path_altitude_ft: Math.round(pt.path_alt_ft),
        open: isOpen,
      };
    });
  }

  const METHOD_B_OFFSET_FPM = { 2000: 1200, 3000: 1300, 4000: 1400 };

  function methodBOffsetAndRate(startInput, defaultOffset) {
    defaultOffset = defaultOffset == null ? 4000 : defaultOffset;
    let offset;
    if (startInput == null || startInput === "") {
      offset = defaultOffset;
    } else {
      const v = Math.round(Number(startInput));
      if (METHOD_B_OFFSET_FPM[v] != null) offset = v;
      else if (v === 12000 || v === 13000 || v === 14000) offset = v - 10000;
      else {
        offset = v >= 10000 ? v - 10000 : v;
        const keys = [2000, 3000, 4000];
        offset = keys.reduce((best, o) => (Math.abs(o - offset) < Math.abs(best - offset) ? o : best));
      }
    }
    if (METHOD_B_OFFSET_FPM[offset] == null) {
      const keys = [2000, 3000, 4000];
      offset = keys.reduce((best, o) => (Math.abs(o - offset) < Math.abs(best - offset) ? o : best));
    }
    return [offset, METHOD_B_OFFSET_FPM[offset]];
  }

  function constraintTypOf(c) {
    if (!c) return "none";
    const typ = (c.typ || c.type || "none").toString().replace(/-/g, "_").replace(/ /g, "_").toLowerCase();
    return typ;
  }

  function entryToFlightOrder(entryWaypoints) {
    if (!entryWaypoints || !entryWaypoints.length) return [];
    const n = entryWaypoints.length;
    const flight = [];
    for (let i = 0; i < n; i++) {
      const src = entryWaypoints[n - 1 - i];
      const wp = Object.assign({}, src);
      if (src.altitude_constraint) wp.altitude_constraint = Object.assign({}, src.altitude_constraint);
      if (i === 0) wp.leg_distance_nm = 0;
      else wp.leg_distance_nm = +(entryWaypoints[n - i].leg_distance_nm || 0);
      flight.push(wp);
    }
    return flight;
  }

  function classifyEnergyState(requiredVsFpm, idleVsFpm, band) {
    if (band == null) band = DEFAULT_ENERGY_IDLE_BAND;
    const req = requiredVsFpm == null ? 0 : Math.max(0, +requiredVsFpm);
    if (idleVsFpm == null || +idleVsFpm <= 0) return req <= 25 ? ENERGY_POWER_ON : ENERGY_ADD_DRAG;
    const idle = +idleVsFpm;
    band = Math.max(0, +band);
    if (req > idle * (1 + band)) return ENERGY_ADD_DRAG;
    if (req < idle * (1 - band)) return ENERGY_POWER_ON;
    return ENERGY_IDLE;
  }

  function annotateConstraintLegs(outputWps, opts) {
    const band = opts.band != null ? +opts.band : DEFAULT_ENERGY_IDLE_BAND;
    const cruiseAlt = opts.cruiseAlt;
    const highEndAlt = opts.highEndAlt;
    for (let i = 0; i < outputWps.length; i++) {
      const cur = outputWps[i];
      if (cur.planned_alt_ft == null) continue;
      let prevName, startAlt, avail;
      if (i === 0) {
        prevName = "TOD";
        startAlt = +cruiseAlt;
        avail = +(cur.distance_from_tod_nm || 0);
      } else {
        const prev = outputWps[i - 1];
        if (prev.planned_alt_ft == null) continue;
        prevName = prev.name;
        startAlt = +prev.planned_alt_ft;
        avail = +(cur.leg_distance_nm || 0);
      }
      const endAlt = +cur.planned_alt_ft;
      const delta = startAlt - endAlt;
      const midAlt = 0.5 * (startAlt + endAlt);
      const prevIas = i ? outputWps[i - 1].planned_ias_kias : null;
      const midIas = cur.planned_ias_kias || prevIas || 250;
      const heldIas = midAlt <= (highEndAlt || 0) + 1 ? midIas : null;
      const st = speedState(midAlt, {
        descent_mach: opts.descentMach,
        high_speed_ias: opts.highSpeedIas,
        ias_held: heldIas,
        isa_dev_c: opts.isaDevC,
      });
      const gs = groundspeedKt(st.tas_kt, opts.wind);
      let reqVs, reqAng;
      if (avail > 0.05) {
        reqVs = (delta / avail) * (gs / 60);
        reqAng = radToDeg(Math.atan(delta / (avail * FT_PER_NM)));
      } else {
        reqVs = Math.abs(delta) <= 50 ? 0 : null;
        reqAng = Math.abs(delta) <= 50 ? 0 : null;
      }
      let idleVs;
      if (midAlt > (highEndAlt || 0) + 1) idleVs = idleVsAtGs(gs, opts.idleAngle, opts.vsCap, st);
      else idleVs = idleVsForIas(midIas, opts.idle250, opts.idle210);
      // v3.0: idle capability = the stay-high planner's own idle simulation over the leg.
      let ceil = null;
      if (opts.stayHighOpts && avail > 0.05 && delta > 50 && !cur.open) {
        const endIas = "ceiling_end_ias" in cur ? cur.ceiling_end_ias : cur.planned_ias_kias;
        const startIas = "ceiling_end_ias" in cur ? cur.ceiling_start_ias : null;
        ceil = Math.min(
          idleStartAltitude(endAlt, endIas, avail, Object.assign({}, opts.stayHighOpts, { startIas: startIas })),
          +cruiseAlt
        );
        idleVs = (Math.max(0, ceil - endAlt) / avail) * (gs / 60);
      }
      let state = classifyEnergyState(reqVs == null ? 0 : reqVs, idleVs, band);
      if (ceil != null && state === ENERGY_ADD_DRAG && startAlt <= ceil + 25) state = ENERGY_IDLE;
      if (cur.open) {
        state = ENERGY_IDLE; // open fix: idle continuation, no target
      } else if (cur.inbound_drag) {
        state = ENERGY_ADD_DRAG;
      } else if (state === ENERGY_POWER_ON && i > 0) {
        const prevW = outputWps[i - 1];
        const prevCeil = prevW.idle_ceiling_ft;
        const prevPlanned = prevW.planned_alt_ft;
        if (prevPlanned == null || prevCeil == null || prevPlanned <= prevCeil + 40) {
          state = ENERGY_STAY_HIGH;
        }
      }
      cur.leg_from = prevName;
      cur.leg_nm = round(avail, 2);
      cur.leg_required_vs_fpm = reqVs == null ? null : round(reqVs, 0);
      cur.leg_path_angle_deg = reqAng == null ? null : round(reqAng, 1);
      cur.leg_idle_vs_fpm = round(idleVs, 0);
      cur.leg_energy_state = state;
      cur.leg_energy_label = ENERGY_LABELS[state];
      cur.leg_gs_kt = round(gs, 1);
    }
  }


  function round(x, n) {
    if (x == null || Number.isNaN(x)) return x;
    const f = Math.pow(10, n);
    return Math.round(x * f) / f;
  }

  function radToDeg(r) {
    return (r * 180) / Math.PI;
  }

  function degToRad(d) {
    return (d * Math.PI) / 180;
  }

  function isHardAt(r) {
    return r.constraint_min_ft != null && r.constraint_max_ft != null && Math.abs(r.constraint_min_ft - r.constraint_max_ft) < 1;
  }

  function plan(inputs) {
    const flags = [];
    flags.push("Primary performance: Ben Irvine, line technique, 2026-11-22.");
    flags.push("The 3 NM per 1,000 ft manual rule of thumb is a reference only and is not used as the performance model.");

    const rates = resolveRates(inputs.weight_class, inputs.landing_weight_lb);
    flags.push.apply(flags, rates.flags);
    const idle250 = rates.idle_250_fpm;
    const idle210 = rates.idle_210_fpm;

    let method = String(inputs.decel_method || "A").trim().toUpperCase();
    if (method !== "A" && method !== "B") throw new Error("decel_method must be 'A' (level-off) or 'B' (shallow descent)");

    const descentMach = inputs.descent_mach != null ? +inputs.descent_mach : DEFAULT_DESCENT_MACH;
    const highSpeedIas = inputs.high_speed_ias_kias != null ? +inputs.high_speed_ias_kias : DEFAULT_HIGH_SPEED_IAS;
    const isaDevC = inputs.isa_dev_c || 0;
    const wind = inputs.wind_component_kt || 0;
    const stepFt = inputs.step_ft != null ? +inputs.step_ft : DEFAULT_STEP_FT;
    const decel210Nm = inputs.decel_210_distance_nm != null ? +inputs.decel_210_distance_nm : DEFAULT_DECEL_210_NM;
    const windowTarget = inputs.window_target || "bottom";
    const cruiseAlt = +inputs.cruise_altitude_ft;

    flags.push("Usual descent Mach is 0.74 (confirmed by Ben Irvine, 2026-11-22); the value remains user-adjustable.");
    flags.push(
      "ASSUMPTION_TO_REFINE: 210 KIAS deceleration is a gradual descending blend over " +
        decel210Nm.toFixed(1) +
        " NM (configurable). Ben has not yet confirmed when to start slowing to 210."
    );

    if (!(highSpeedIas >= DESCENT_IAS_MIN_KIAS - 1e-6 && highSpeedIas <= DESCENT_IAS_MAX_KIAS + 1e-6)) {
      throw new Error("Descent IAS must be " + DESCENT_IAS_MIN_KIAS.toFixed(0) + "–" + DESCENT_IAS_MAX_KIAS.toFixed(0) + " KIAS, got " + highSpeedIas.toFixed(0) + ".");
    }
    const xo = crossoverAltitudeFt(descentMach, highSpeedIas, isaDevC);

    const cruiseSt = speedState(cruiseAlt, { descent_mach: descentMach, high_speed_ias: highSpeedIas, isa_dev_c: isaDevC });
    const cruiseGs = groundspeedKt(cruiseSt.tas_kt, wind);
    const idleAngle = makeIdleModel(rates.class, idle250, inputs.idle_path_angle_deg != null ? +inputs.idle_path_angle_deg : DEFAULT_IDLE_PATH_ANGLE_DEG);
    const idleSummary = idleAngle.summary();
    const vsCap = inputs.tod_vs_cap_fpm != null ? +inputs.tod_vs_cap_fpm : DEFAULT_TOD_VS_CAP_FPM;
    const uncappedTodVs = idleAngle.vsFpm(cruiseSt);
    const todVs = idleVsAtGs(cruiseGs, idleAngle, vsCap, cruiseSt);
    const capAppliedAtTod = todVs + 0.5 < uncappedTodVs;
    const todAngle = angleFromVsGs(todVs, cruiseGs);
    const todAngleDeg = radToDeg(todAngle);
    const idleAngleDeg = radToDeg(idleAngle.angleRad(cruiseSt));
    const hsLabel = "M" + descentMach.toFixed(2) + "/" + highSpeedIas.toFixed(0);
    if (idleAngle.fixedAngleDeg != null) {
      flags.push("Idle " + hsLabel + " path forced to a fixed " + idleAngle.fixedAngleDeg.toFixed(1) + "° air-mass angle; VS = TAS × tan(angle), capped at " + vsCap.toFixed(0) + " fpm.");
    } else {
      flags.push(
        "Idle " + hsLabel + " model (Ben's 2026-11-23 flights, 3 legs only): " + idleAngle.machRefVs.toFixed(0) +
          " fpm at M0.74/FL350 for " + rates.class + " (" + idleSummary.mach_angle_deg.toFixed(2) + "° air-mass) in the Mach segment; " +
          radToDeg(idleAngle.iasAngleRad(highSpeedIas)).toFixed(2) + "° at " + highSpeedIas.toFixed(0) +
          " KIAS below the crossover. VS = TAS × tan(angle) so it falls as TAS drops; capped at " + vsCap.toFixed(0) + " fpm."
      );
    }

    const wps = inputs.waypoints || [];
    if (!wps.length) throw new Error("at least one waypoint is required");

    const resolved = [];
    let cum = 0;
    for (let i = 0; i < wps.length; i++) {
      const wp = wps[i];
      cum += Math.max(0, +wp.leg_distance_nm);
      const band = constraintBand(wp.altitude_constraint, windowTarget);
      resolved.push({
        index: i,
        name: wp.name,
        leg_distance_nm: +wp.leg_distance_nm,
        distance_from_start_nm: cum,
        constraint_target_ft: band[0],
        constraint_min_ft: band[1],
        constraint_max_ft: band[2],
        speed_constraint: wp.speed_constraint != null && wp.speed_constraint !== "" ? +wp.speed_constraint : null,
        planned_alt_ft: band[0],
        planned_ias_kias: wp.speed_constraint != null && wp.speed_constraint !== "" ? +wp.speed_constraint : null,
        required_vs_fpm: null,
        constraint_typ: constraintTypOf(wp.altitude_constraint),
        synthetic: false,
        notes: [],
      });
    }

    const goesBelow10k = resolved.some((r) => {
      return (
        (r.constraint_target_ft != null && r.constraint_target_ft <= 10000) ||
        (r.constraint_max_ft != null && r.constraint_max_ft <= 10000) ||
        (r.speed_constraint != null && r.speed_constraint <= 250 && r.constraint_target_ft != null && r.constraint_target_ft <= 12000)
      );
    });

    // v3.0: the plan ends at the last fix with an altitude restriction. Later fixes
    // with no altitude are OPEN (no invented target, never a drag flag). See planner.py.
    const altIdx = [];
    resolved.forEach((r, i) => {
      if ((r.constraint_typ || "none") !== "none" &&
          (r.constraint_target_ft != null || r.constraint_min_ft != null || r.constraint_max_ft != null)) altIdx.push(i);
    });
    if (!altIdx.length) throw new Error("Set an altitude restriction on at least one waypoint (the plan needs an altitude to descend to).");
    const endIdx = altIdx[altIdx.length - 1];
    const fieldElev = inputs.field_elevation_ft == null || inputs.field_elevation_ft === "" ? null : +inputs.field_elevation_ft;
    const openNames = resolved.slice(endIdx + 1).map((r) => r.name);
    for (const r of resolved.slice(endIdx + 1)) {
      r.open = true;
      r.planned_alt_ft = null;
    }
    if (openNames.length) {
      flags.push(
        "OPEN_LAST_FIX: " + openNames.join(", ") + " " + (openNames.length === 1 ? "has" : "have") +
          " no altitude restriction, so the plan ends at " + resolved[endIdx].name +
          " and does not invent a target. Set an altitude on the last fix if you want the plan to aim for one."
      );
    }
    function openEstimates() {
      let prev = resolved[endIdx];
      const floor = fieldElev != null ? fieldElev : 0;
      for (const r of resolved.slice(endIdx + 1)) {
        const pAlt = +prev.planned_alt_ft;
        const iasA = prev.planned_ias_kias;
        const iasB = r.planned_ias_kias || iasA;
        const leg = Math.max(+(r.leg_distance_nm || 0), 0);
        let st, vs;
        if (pAlt > 10000 + 1 || iasB == null) {
          st = speedState(pAlt, { descent_mach: descentMach, high_speed_ias: highSpeedIas, isa_dev_c: isaDevC });
          vs = idleVsAtGs(groundspeedKt(st.tas_kt, wind), idleAngle, vsCap, st);
        } else {
          const iasMid = 0.5 * (Math.min(+(iasA || 250), 250) + Math.min(+iasB, 250));
          st = speedState(pAlt, { descent_mach: descentMach, high_speed_ias: highSpeedIas, ias_held: iasMid, isa_dev_c: isaDevC });
          vs = idleVsForIas(iasMid, idle250, idle210);
        }
        const gs = groundspeedKt(st.tas_kt, wind);
        const est = Math.max(floor, pAlt - (leg * vs * 60) / Math.max(gs, MIN_GS_KT));
        r.planned_alt_ft = est;
        r.open_idle_estimate_ft = est;
        prev = r;
      }
    }

    // Stay-high crossings: work backward from the last constraint at idle
    // capability (including method A/B and 210/intermediate decels).
    flags.push(
      "Stay-high planning: each crossing is the highest altitude that still allows an idle descent " +
        "(with deceleration segments) to later restrictions. Drag is shown only when even the window " +
        "bottom or hard restriction is too high to make a later fix at idle."
    );
    let inheritedSpd = null;
    for (const r of resolved) {
      if (r.speed_constraint != null) {
        r.planned_ias_kias = r.speed_constraint;
        inheritedSpd = r.speed_constraint;
      } else {
        r.planned_ias_kias = inheritedSpd;
      }
    }

    let prelimGateIdx = null;
    let prelimGateAlt = 10000;
    for (let i = 0; i < resolved.length; i++) {
      const r = resolved[i];
      const [lo, hi] = constraintFloorCeiling(r);
      const spd = r.speed_constraint;
      const altGuess = hi != null ? hi : lo != null ? lo : r.constraint_target_ft;
      if (spd != null && spd <= 250 && (altGuess == null || altGuess >= 10000 - 1)) {
        prelimGateIdx = i;
        prelimGateAlt = +(hi != null ? hi : altGuess != null ? altGuess : 10000);
        break;
      }
      if (altGuess != null && Math.abs(altGuess - 10000) <= 50 && (spd == null || spd <= 250)) {
        prelimGateIdx = i;
        prelimGateAlt = 10000;
        break;
      }
    }

    const methodAKtNmPre = PERF.method_a_kt_per_nm;
    let preBOffset = 0, preBVs = null;
    if (method !== "A") {
      const br = methodBOffsetAndRate(inputs.method_b_start_altitude_ft, 4000);
      preBOffset = +br[0];
      preBVs = br[1];
    }

    function placeStayHigh(gateAlt) {
      for (const r of resolved) {
        r.inbound_drag = false;
        r.drag_leg = false;
        r.drag_note = null;
        r.idle_ceiling_ft = null;
      }
      for (const r of resolved.slice(endIdx + 1)) r.planned_alt_ft = null;
      const lastR = resolved[endIdx];
      let [lo, hi] = constraintFloorCeiling(lastR);
      let typ = lastR.constraint_typ || "none";
      if (typ === "window" && hi != null) lastR.planned_alt_ft = hi;
      else if (typ === "at") lastR.planned_alt_ft = lo != null ? lo : hi;
      else if (typ === "at_or_below" && hi != null) lastR.planned_alt_ft = hi;
      else if (typ === "at_or_above" && lo != null && (lastR.planned_alt_ft == null || lastR.planned_alt_ft < lo)) lastR.planned_alt_ft = lo;
      for (let i = endIdx - 1; i >= 0; i--) {
        const nxt = resolved[i + 1];
        const cur = resolved[i];
        const leg = Math.max(+(nxt.leg_distance_nm || 0), 0);
        const nxtAlt = +nxt.planned_alt_ft;
        const nxtIas = nxt.planned_ias_kias;
        nxt.ceiling_end_ias = nxtIas;
        nxt.ceiling_start_ias = cur.planned_ias_kias;
        let ceil = idleStartAltitude(nxtAlt, nxtIas, leg, {
          startIas: cur.planned_ias_kias,
          inputs: null,
          idleAngle: idleAngle,
          vsCap: vsCap,
          idle250: idle250,
          idle210: idle210,
          method: method,
          gateAlt: gateAlt,
          methodBOffset: preBOffset,
          methodBVs: preBVs,
          methodAKtNm: methodAKtNmPre,
          decelBaseNm: +(inputs.decel_210_distance_nm != null ? inputs.decel_210_distance_nm : 5),
          descentMach: descentMach,
          highSpeedIas: highSpeedIas,
          isaDevC: isaDevC,
          wind: wind,
          cruiseAlt: cruiseAlt,
        });
        ceil = Math.min(ceil, +cruiseAlt);
        cur.idle_ceiling_ft = ceil;
        [lo, hi] = constraintFloorCeiling(cur);
        typ = cur.constraint_typ || "none";
        let planned;
        if (typ === "at") planned = lo != null ? lo : hi;
        else if (typ === "window") {
          planned = hi != null ? Math.min(hi, ceil) : ceil;
          if (lo != null) planned = Math.max(planned, lo);
        } else if (typ === "at_or_below") planned = hi != null ? Math.min(hi, ceil) : ceil;
        else if (typ === "at_or_above") {
          planned = ceil;
          if (lo != null) planned = Math.max(planned, lo);
        } else planned = ceil;
        let floor = null;
        if (typ === "at" || typ === "window" || typ === "at_or_above") {
          floor = typ === "at" ? (lo != null ? lo : hi) : lo;
        }
        if (floor != null && floor > ceil + 25) {
          planned = floor;
          const kind = typ === "window" ? "window bottom" : "restriction";
          const note =
            cur.name +
            " to " +
            nxt.name +
            ": add drag. Even the " +
            kind +
            " at " +
            floor.toFixed(0) +
            " ft is too high to idle to " +
            nxt.name +
            " at " +
            nxtAlt.toFixed(0) +
            " ft (idle ceiling " +
            ceil.toFixed(0) +
            " ft).";
          cur.notes.push(note);
          flags.push(note);
          cur.drag_leg = true;
          nxt.inbound_drag = true;
          nxt.drag_note = note;
        }
        cur.planned_alt_ft = planned;
        let n = "Stay-high crossing " + planned.toFixed(0) + " ft (idle ceiling " + ceil.toFixed(0) + " ft";
        if (typ === "window" && lo != null && hi != null) n += ", window " + lo.toFixed(0) + "–" + hi.toFixed(0) + " ft";
        n += ").";
        cur.notes.push(n);
      }
      for (let i = 1; i < resolved.length; i++) {
        const prevA = resolved[i - 1].planned_alt_ft;
        const cur = resolved[i];
        typ = cur.constraint_typ || "none";
        [lo, hi] = constraintFloorCeiling(cur);
        if (prevA == null || cur.planned_alt_ft == null) continue;
        if ((typ === "none" || typ === "at_or_below") && cur.planned_alt_ft > prevA + 1) {
          cur.planned_alt_ft = prevA;
          cur.notes.push("Held at " + prevA.toFixed(0) + " ft (cannot plan a climb after " + resolved[i - 1].name + ").");
          cur.held_level_from = resolved[i - 1].name;
          cur.held_level_alt = prevA;
        } else if (typ === "window" && lo != null && lo <= prevA + 1) {
          const capped = Math.min(cur.planned_alt_ft, prevA);
          cur.planned_alt_ft = Math.max(capped, lo);
        }
      }
      openEstimates();
    }

    placeStayHigh(prelimGateAlt);
    if (prelimGateIdx != null && resolved[prelimGateIdx].planned_alt_ft != null) {
      prelimGateAlt = +resolved[prelimGateIdx].planned_alt_ft;
    }
    placeStayHigh(prelimGateAlt);
    // The exact idle model the stay-high planner used; reused by the drag checks.
    const stayHighOpts = {
      inputs: null,
      idleAngle: idleAngle,
      vsCap: vsCap,
      idle250: idle250,
      idle210: idle210,
      method: method,
      gateAlt: prelimGateAlt,
      methodBOffset: preBOffset,
      methodBVs: preBVs,
      methodAKtNm: methodAKtNmPre,
      decelBaseNm: +(inputs.decel_210_distance_nm != null ? inputs.decel_210_distance_nm : 5),
      descentMach: descentMach,
      highSpeedIas: highSpeedIas,
      isaDevC: isaDevC,
      wind: wind,
      cruiseAlt: cruiseAlt,
    };

    // Planned speeds along the path. After a speed restriction is met, later
    // Planned speeds along the path. After a speed restriction is met, later
    // fixes inherit it (so the runway after a 210 limit is not reset to 250).
    let inherited = null;
    for (const r of resolved) {
      if (r.speed_constraint != null) {
        r.planned_ias_kias = r.speed_constraint;
        inherited = r.speed_constraint;
      } else if (r.planned_alt_ft != null && r.planned_alt_ft <= 10000) {
        r.planned_ias_kias = inherited != null && inherited <= 250 ? inherited : 250;
        if (inherited == null) inherited = 250;
      } else {
        r.planned_ias_kias = inherited;
      }
    }

    // 250 KIAS / 10,000 ft is always a constraint. A user restriction can
    // tighten it: a 250 (or slower) limit above 10,000 ft must be met there,
    // or a slower speed at 10,000 ft replaces 250.
    let first250Idx = null;
    let alt250 = 10000;
    let speedAt250Gate = 250;
    for (let i = 0; i < resolved.length; i++) {
      const r = resolved[i];
      const alt = r.planned_alt_ft;
      const spd = r.speed_constraint;
      if (alt == null) continue;
      if (spd != null && spd <= 250 && alt >= 10000 - 1) {
        first250Idx = i;
        alt250 = +alt;
        speedAt250Gate = 250;
        break;
      }
      if (Math.abs(alt - 10000) <= 50) {
        first250Idx = i;
        alt250 = 10000;
        if (spd != null && spd < 250) speedAt250Gate = +spd;
        break;
      }
    }
    flags.push(
      "Built-in constraint: 250 KIAS at 10,000 ft" +
        (first250Idx != null && alt250 > 10000 + 1
          ? ", met at " + resolved[first250Idx].name + " (" + alt250.toFixed(0) + " ft / 250 KIAS)."
          : ".")
    );
    if (first250Idx != null) {
      if (resolved[first250Idx].planned_ias_kias == null || resolved[first250Idx].planned_ias_kias > 250) {
        resolved[first250Idx].planned_ias_kias = 250;
      }
    }

    const speedTargets = [];
    let prevSpd = 250;
    for (let i = 0; i < resolved.length; i++) {
      const r = resolved[i];
      if (first250Idx != null && i <= first250Idx) continue;
      if (r.planned_alt_ft == null) continue;
      if (r.planned_alt_ft > alt250 + 1) continue;
      const spd = r.speed_constraint;
      if (spd != null && spd < prevSpd - 0.5) {
        speedTargets.push({ idx: i, name: r.name, alt_ft: +r.planned_alt_ft, speed_kias: +spd });
        prevSpd = +spd;
      }
    }
    const first210Idx = (() => {
      const t = speedTargets.find((x) => x.speed_kias <= 210);
      return t ? t.idx : null;
    })();

    const methodAKtNm = PERF.method_a_kt_per_nm;
    const decelSegments = [];
    const at250Name = first250Idx != null ? resolved[first250Idx].name : "10,000 ft";

    let highEndAlt = null;
    let methodBDist = 0;
    let methodBStart = null;
    let methodBVs = null;
    let dLevel = 0;
    let usedLevelFallback = false;

    if (method === "A") {
      dLevel = methodALevelDistanceNm(highSpeedIas, 250, methodAKtNm);
      highEndAlt = alt250;
      decelSegments.push({
        type: "method_A_level_off",
        from_kias: highSpeedIas,
        to_kias: 250,
        altitude_ft: alt250,
        end_altitude_ft: alt250,
        start_altitude_ft: alt250,
        distance_nm: dLevel,
        kt_per_nm: methodAKtNm,
        before_fix: at250Name,
        vs_fpm: 0,
        source: "Ben Irvine, line technique, 2026-11-22",
        note:
          "Level off at " +
          alt250.toFixed(0) +
          " ft, " +
          dLevel.toFixed(1) +
          " NM before " +
          at250Name +
          ", and lose about " +
          methodAKtNm.toFixed(0) +
          " kt per NM, to be at 250 KIAS by " +
          at250Name +
          ".",
      });
      flags.push(
        "Deceleration to 250 KIAS: method A level-off, " +
          dLevel.toFixed(1) +
          " NM before " +
          at250Name +
          " at " +
          alt250.toFixed(0) +
          " ft, " +
          methodAKtNm.toFixed(0) +
          " kt/NM."
      );
    } else {
      const offRate = methodBOffsetAndRate(inputs.method_b_start_altitude_ft, 4000);
      const offset = offRate[0];
      methodBVs = offRate[1];
      methodBStart = alt250 + offset;
      if (methodBStart >= cruiseAlt) {
        methodBStart = Math.max(alt250 + 2000, cruiseAlt - 500);
      }
      if (methodBStart <= alt250) {
        usedLevelFallback = true;
        dLevel = methodALevelDistanceNm(highSpeedIas, 250, methodAKtNm);
        highEndAlt = alt250;
        methodBStart = null;
        methodBVs = null;
        decelSegments.push({
          type: "method_A_level_off",
          from_kias: highSpeedIas,
          to_kias: 250,
          altitude_ft: alt250,
          end_altitude_ft: alt250,
          start_altitude_ft: alt250,
          distance_nm: dLevel,
          kt_per_nm: methodAKtNm,
          before_fix: at250Name,
          vs_fpm: 0,
          source: "Ben Irvine, line technique, 2026-11-22",
          note:
            "Method B window was not above " +
            at250Name +
            " (" +
            alt250.toFixed(0) +
            " ft); level off " +
            dLevel.toFixed(1) +
            " NM before the fix instead.",
        });
        flags.push(
          "Method B could not start above " +
            alt250.toFixed(0) +
            " ft; using a level-off " +
            dLevel.toFixed(1) +
            " NM before " +
            at250Name +
            "."
        );
        method = "A";
      } else {
        highEndAlt = methodBStart;
        const integB = integrateConstantVs(methodBStart, alt250, 0.5 * (highSpeedIas + 250), methodBVs, {
          isa_dev_c: isaDevC,
          wind_component_kt: wind,
          step_ft: stepFt,
          label: "method_B_shallow",
        });
        methodBDist = integB.distance_nm;
        decelSegments.push({
          type: "method_B_shallow_descent",
          from_kias: highSpeedIas,
          to_kias: 250,
          start_altitude_ft: methodBStart,
          end_altitude_ft: alt250,
          vs_fpm: methodBVs,
          distance_nm: round(methodBDist, 2),
          before_fix: at250Name,
          source: "Ben Irvine, line technique, 2026-11-22",
          note:
            "Shallow at " +
            methodBStart.toFixed(0) +
            " ft (" +
            offset.toFixed(0) +
            " ft above the 250 constraint), " +
            methodBVs.toFixed(0) +
            " fpm, for about " +
            methodBDist.toFixed(1) +
            " NM to reach 250 KIAS by " +
            at250Name +
            " (" +
            alt250.toFixed(0) +
            " ft).",
          flag: "method_B_decel_distance_from_altitude_window",
        });
        flags.push(
          "Deceleration to 250 KIAS: method B shallow " +
            methodBVs.toFixed(0) +
            " fpm from " +
            methodBStart.toFixed(0) +
            " ft to " +
            alt250.toFixed(0) +
            " ft (" +
            methodBDist.toFixed(1) +
            " NM), finishing at " +
            at250Name +
            "."
        );
      }
    }

    const gradualIntegs = [];
    let alt210Start = null, alt210End = null;
    let fromSpd = 250;
    let fromAlt = alt250;
    let fromIdx = first250Idx;
    for (const tgt of speedTargets) {
      const toSpd = tgt.speed_kias;
      const toAlt = tgt.alt_ft;
      const vsTop = idleVsForIas(fromSpd, idle250, idle210);
      const vsBot = idleVsForIas(toSpd, idle250, idle210);
      const wantNm = gradualDecelDistanceNm(fromSpd, toSpd, decel210Nm);
      let avail = wantNm;
      if (fromIdx != null) {
        avail = 0;
        for (let j = fromIdx + 1; j <= tgt.idx; j++) avail += resolved[j].leg_distance_nm;
      }
      let useNm = wantNm;
      if (avail > 0) useNm = Math.min(wantNm, Math.max(avail, 0.5));
      let lo = toAlt, hi = Math.max(toAlt + 100, Math.min(fromAlt - 50, cruiseAlt - 500));
      if (hi <= lo) hi = lo + 500;
      let integ = null;
      for (let k = 0; k < 30; k++) {
        const mid = 0.5 * (lo + hi);
        const midIas = 0.5 * (fromSpd + toSpd);
        const trial = integrateConstantVs(mid, toAlt, midIas, vsTop, {
          isa_dev_c: isaDevC,
          wind_component_kt: wind,
          step_ft: stepFt,
          label: "decel_" + fromSpd.toFixed(0) + "_to_" + toSpd.toFixed(0),
          vs_top: vsTop,
          vs_bottom: vsBot,
        });
        if (trial.distance_nm > useNm) hi = mid;
        else lo = mid;
        integ = trial;
      }
      let startAlt = 0.5 * (lo + hi);
      if (startAlt <= toAlt + 10) {
        startAlt = Math.min(fromAlt - 50, toAlt + Math.max(300, fromAlt > toAlt ? (fromAlt - toAlt) * 0.3 : 300));
        if (startAlt <= toAlt) startAlt = toAlt + 300;
      }
      integ = integrateConstantVs(startAlt, toAlt, 0.5 * (fromSpd + toSpd), vsTop, {
        isa_dev_c: isaDevC,
        wind_component_kt: wind,
        step_ft: stepFt,
        label: "decel_" + fromSpd.toFixed(0) + "_to_" + toSpd.toFixed(0),
        vs_top: vsTop,
        vs_bottom: vsBot,
      });
      let dnm = integ.distance_nm;
      if (dnm < 0.4) {
        dnm = Math.max(useNm, 2);
        integ.distance_nm = dnm;
      }
      const typ = toSpd <= 210 + 0.5 ? "gradual_decel_to_210" : "gradual_decel";
      decelSegments.push({
        type: typ,
        from_kias: fromSpd,
        to_kias: toSpd,
        start_altitude_ft: round(startAlt, 0),
        end_altitude_ft: toAlt,
        vs_start_fpm: vsTop,
        vs_end_fpm: vsBot,
        distance_nm: round(dnm, 2),
        requested_distance_nm: wantNm,
        before_fix: tgt.name,
        source: "ASSUMPTION_TO_REFINE",
        note:
          "Start slowing from " +
          fromSpd.toFixed(0) +
          " to " +
          toSpd.toFixed(0) +
          " KIAS at about " +
          startAlt.toFixed(0) +
          " ft, " +
          dnm.toFixed(1) +
          " NM before " +
          tgt.name +
          ". Ease VS from " +
          vsTop.toFixed(0) +
          " to " +
          vsBot.toFixed(0) +
          " fpm (idle rate interpolated by speed). Distance is an assumption to refine.",
        flag: "ASSUMPTION_TO_REFINE",
      });
      gradualIntegs.push({
        start_alt: startAlt,
        end_alt: toAlt,
        from_spd: fromSpd,
        to_spd: toSpd,
        integ: integ,
        vs_top: vsTop,
        vs_bot: vsBot,
        idx: tgt.idx,
      });
      if (toSpd <= 210 + 0.5) {
        alt210Start = startAlt;
        alt210End = toAlt;
      }
      fromSpd = toSpd;
      fromAlt = toAlt;
      fromIdx = tgt.idx;
    }

    if (highEndAlt == null) {
      highEndAlt = resolved[0].planned_alt_ft || alt250 || cruiseAlt - 5000;
    }

    // If already at or below the high-speed end, skip the idle Mach/290 path
    let highInteg = { distance_nm: 0, steps: [], cap_applied: false, vs_start_fpm: null, start_angle_deg: null };
    if (cruiseAlt > highEndAlt + 1) {
      highInteg = integrateIdlePath(cruiseAlt, highEndAlt, idleAngle, {
        descent_mach: descentMach,
        high_speed_ias: highSpeedIas,
        isa_dev_c: isaDevC,
        wind_component_kt: wind,
        step_ft: stepFt,
        vs_cap_fpm: vsCap,
        label: "high_speed_idle",
      });
    }
    const dHigh = highInteg.distance_nm;
    if (highInteg.cap_applied) {
      flags.push(
        "Idle rate would exceed the " +
          vsCap.toFixed(0) +
          " fpm cap; initial path is " +
          highInteg.start_angle_deg.toFixed(2) +
          "° at " +
          highInteg.vs_start_fpm.toFixed(0) +
          " fpm until the cap no longer binds."
      );
    }

    let antiIceNm = 0;
    if (inputs.anticipated_anti_ice_ft > 0) {
      antiIceNm = 2.0 * (inputs.anticipated_anti_ice_ft / 1000);
      flags.push(
        "TOD advanced " +
          antiIceNm.toFixed(1) +
          " NM for anticipated anti-ice (" +
          inputs.anticipated_anti_ice_ft.toFixed(0) +
          " ft × 2 NM / 1,000 ft, AOM Vol 2 2.7-2)."
      );
    }
    const todToHighEnd = dHigh + antiIceNm;

    const remainingIntegs = [];
    let cursorAlt = alt250;
    let cursorSpd = 250;
    for (const g of gradualIntegs) {
      if (g.start_alt < cursorAlt - 10) {
        remainingIntegs.push(
          integrateConstantVs(cursorAlt, g.start_alt, cursorSpd, idleVsForIas(cursorSpd, idle250, idle210), {
            isa_dev_c: isaDevC,
            wind_component_kt: wind,
            step_ft: stepFt,
            label: "idle_" + cursorSpd.toFixed(0),
          })
        );
      }
      remainingIntegs.push(g.integ);
      cursorAlt = g.end_alt;
      cursorSpd = g.to_spd;
    }
    const lastAlt = resolved[resolved.length - 1].planned_alt_ft;
    if (lastAlt != null && lastAlt < cursorAlt - 10) {
      remainingIntegs.push(
        integrateConstantVs(cursorAlt, lastAlt, cursorSpd, idleVsForIas(cursorSpd, idle250, idle210), {
          isa_dev_c: isaDevC,
          wind_component_kt: wind,
          step_ft: stepFt,
          label: "idle_" + cursorSpd.toFixed(0),
        })
      );
    }

    const assembled = [];
    function extendSteps(steps, newSegStartNm) {
      let acc = newSegStartNm;
      for (const s of steps) {
        acc = newSegStartNm + s.cumulative_distance_nm;
        const row = Object.assign({}, s);
        row.distance_from_tod_nm = round(acc, 3);
        assembled.push(row);
      }
      return acc;
    }

    let cursor = antiIceNm || 0;
    cursor = extendSteps(highInteg.steps || [], cursor);
    const highEndDist = cursor;
    if (method === "A" && dLevel) {
      const st = speedState(highEndAlt, {
        descent_mach: descentMach,
        high_speed_ias: highSpeedIas,
        ias_held: 0.5 * (highSpeedIas + 250),
        isa_dev_c: isaDevC,
      });
      const gs = groundspeedKt(st.tas_kt, wind);
      cursor += dLevel;
      assembled.push({
        alt_top_ft: highEndAlt,
        alt_bottom_ft: highEndAlt,
        altitude_ft: highEndAlt,
        ias_kias: 0.5 * (highSpeedIas + 250),
        mach: round(st.mach, 3),
        tas_kt: round(st.tas_kt, 1),
        gs_kt: round(gs, 1),
        vs_fpm: 0,
        distance_nm: round(dLevel, 3),
        distance_from_tod_nm: round(cursor, 3),
        segment: "method_A_level_off",
        regime: "level_decel",
      });
    } else if (method === "B" && methodBStart != null) {
      cursor = extendSteps(
        integrateConstantVs(methodBStart, alt250, 0.5 * (highSpeedIas + 250), methodBVs, {
          isa_dev_c: isaDevC,
          wind_component_kt: wind,
          step_ft: stepFt,
          label: "method_B_shallow",
        }).steps,
        cursor
      );
    }
    for (const integ of remainingIntegs) {
      cursor = extendSteps(integ.steps || [], cursor);
    }
    const totalProfileNm = cursor;

    function profileAtDistance(distNm) {
      if (!assembled.length) return [null, null, null];
      for (const s of assembled) {
        const dBot = s.distance_from_tod_nm;
        const dTop = dBot - s.distance_nm;
        if (distNm <= dBot + 0.0001) {
          if (s.distance_nm > 0 && distNm >= dTop) {
            const frac = (distNm - dTop) / s.distance_nm;
            const alt = s.alt_top_ft != null && s.alt_bottom_ft != null ? s.alt_top_ft + frac * (s.alt_bottom_ft - s.alt_top_ft) : s.altitude_ft;
            return [alt, s.vs_fpm, s];
          }
          return [s.altitude_ft, s.vs_fpm, s];
        }
      }
      const lastS = assembled[assembled.length - 1];
      return [lastS.altitude_ft, lastS.vs_fpm, lastS];
    }

    function matchDistanceByAltitude(alt) {
      if (alt == null || !assembled.length) return null;
      const nearest = assembled.reduce((best, s) => (Math.abs(s.altitude_ft - alt) < Math.abs(best.altitude_ft - alt) ? s : best));
      return nearest.distance_from_tod_nm;
    }

    const decel250Nm = method === "A" ? dLevel : methodBDist;
    const anchorDist = highEndDist + decel250Nm;
    if (first250Idx != null && resolved[first250Idx].planned_alt_ft != null) {
      resolved[first250Idx].distance_from_tod_nm = anchorDist;
      for (let i = first250Idx - 1; i >= 0; i--) {
        const nxtD = resolved[i + 1].distance_from_tod_nm;
        if (nxtD == null) continue;
        resolved[i].distance_from_tod_nm = nxtD - resolved[i + 1].leg_distance_nm;
      }
      for (let i = first250Idx + 1; i < resolved.length; i++) {
        const prevD = resolved[i - 1].distance_from_tod_nm;
        if (prevD == null) continue;
        resolved[i].distance_from_tod_nm = prevD + resolved[i].leg_distance_nm;
      }
    } else if (resolved.length) {
      resolved[0].distance_from_tod_nm = matchDistanceByAltitude(resolved[0].planned_alt_ft) || 0;
      for (let i = 1; i < resolved.length; i++) {
        const prevD = resolved[i - 1].distance_from_tod_nm || 0;
        resolved[i].distance_from_tod_nm = prevD + resolved[i].leg_distance_nm;
      }
    }

    // Crossings are already placed by the stay-high backward pass.
    // Do not re-snap windows or typ-none fixes onto the continuous early energy path.

    // No planned climbs: unconstrained / backward-computed altitudes must be
    // monotonic non-increasing in flying order.
    for (let i = 1; i < resolved.length; i++) {
      const cur = resolved[i], prev = resolved[i - 1];
      if (cur.planned_alt_ft == null || prev.planned_alt_ft == null) continue;
      const typ = cur.constraint_typ || "";
      if (cur.planned_alt_ft <= prev.planned_alt_ft + 1) continue;
      if (typ === "at") {
        flags.push(
          prev.name +
            " to " +
            cur.name +
            ": planned climb from " +
            prev.planned_alt_ft.toFixed(0) +
            " to " +
            cur.planned_alt_ft.toFixed(0) +
            " ft (published 'at' restriction)."
        );
        continue;
      }
      const floor = cur.constraint_min_ft;
      if ((typ === "window" || typ === "at_or_above") && floor != null && floor > prev.planned_alt_ft + 25) {
        flags.push(
          prev.name +
            " to " +
            cur.name +
            ": published minimum " +
            floor.toFixed(0) +
            " ft is above " +
            prev.name +
            " at " +
            prev.planned_alt_ft.toFixed(0) +
            " ft."
        );
        continue;
      }
      let held = prev.planned_alt_ft;
      if (floor != null) held = Math.max(held, floor);
      if (held > prev.planned_alt_ft + 1) continue;
      cur.notes.push("Held at " + held.toFixed(0) + " ft (cannot plan a climb after " + prev.name + ").");
      cur.planned_alt_ft = held;
      cur.held_level_from = prev.name;
      cur.held_level_alt = held;
    }


    function vsForWaypoint(alt, ias) {
      if (alt > (highEndAlt || 0) + 1) {
        const st = speedState(alt, { descent_mach: descentMach, high_speed_ias: highSpeedIas, isa_dev_c: isaDevC });
        const gs = groundspeedKt(st.tas_kt, wind);
        return round(idleVsAtGs(gs, idleAngle, vsCap, st), 0);
      }
      if (method === "B" && methodBStart != null && alt250 != null && alt250 - 1 < alt && alt <= methodBStart + 1) {
        return methodBVs;
      }
      for (const g of gradualIntegs) {
        if (g.end_alt - 1 <= alt && alt <= g.start_alt + 1) {
          if (g.start_alt !== g.end_alt) {
            const frac = (g.start_alt - alt) / (g.start_alt - g.end_alt);
            return round(g.vs_top + frac * (g.vs_bot - g.vs_top), 0);
          }
          return round(g.vs_bot, 0);
        }
      }
      if (ias) return round(idleVsForIas(ias, idle250, idle210), 0);
      return null;
    }

    for (const r of resolved) {
      const alt = r.planned_alt_ft;
      if (alt == null) continue;
      let st;
      if (r.planned_ias_kias != null) {
        st = speedState(alt, {
          descent_mach: descentMach,
          high_speed_ias: highSpeedIas,
          ias_held: r.planned_ias_kias,
          isa_dev_c: isaDevC,
        });
      } else {
        st = speedState(alt, { descent_mach: descentMach, high_speed_ias: highSpeedIas, isa_dev_c: isaDevC });
        r.planned_ias_kias = round(st.ias_kias, 1);
      }
      const gs = groundspeedKt(st.tas_kt, wind);
      r.required_vs_fpm = vsForWaypoint(alt, r.planned_ias_kias);
      if (alt > (highEndAlt || 0) + 1) {
        r.notes.push("Required VS at idle on the " + hsLabel + " model (capped at " + vsCap.toFixed(0) + " fpm).");
      }
      r.tas_kt = round(st.tas_kt, 1);
      r.gs_kt = round(gs, 1);
      r.mach = round(st.mach, 3);
      if (r.distance_from_tod_nm != null) r.distance_from_tod_nm = round(r.distance_from_tod_nm, 2);
      if (r.planned_alt_ft != null) r.planned_alt_ft = round(r.planned_alt_ft, 0);
    }

    function steepMessage(fromName, toName, startAlt, endAlt, avail, need, reqVs, idleVs, reqAng, idleAng, earlier) {
      return (
        fromName +
        " to " +
        toName +
        ": too steep for an idle descent. Required " +
        reqVs.toFixed(0) +
        " fpm (" +
        reqAng.toFixed(1) +
        "°) over " +
        avail.toFixed(1) +
        " NM to lose " +
        startAlt.toFixed(0) +
        "–" +
        endAlt.toFixed(0) +
        " ft; idle capability is " +
        idleVs.toFixed(0) +
        " fpm (" +
        idleAng.toFixed(1) +
        "°). Start down " +
        earlier.toFixed(1) +
        " NM earlier, or level off sooner, or use speed brakes."
      );
    }

    for (let i = 0; i < resolved.length - 1; i++) {
      const cur = resolved[i], nxt = resolved[i + 1];
      if (cur.planned_alt_ft == null || nxt.planned_alt_ft == null) continue;
      const startAlt = cur.planned_alt_ft;
      const endAlt = nxt.planned_alt_ft;
      if (startAlt <= endAlt + 50) continue;
      if (nxt.open) continue; // open fix: no target, never a drag flag
      const avail = Math.max(nxt.leg_distance_nm, 0);
      if (avail <= 0.05) continue;
      const midAlt = 0.5 * (startAlt + endAlt);
      const midIas = nxt.planned_ias_kias || cur.planned_ias_kias || 250;
      const st = speedState(midAlt, {
        descent_mach: descentMach,
        high_speed_ias: highSpeedIas,
        ias_held: midAlt <= (highEndAlt || 0) + 1 ? midIas : null,
        isa_dev_c: isaDevC,
      });
      const gs = groundspeedKt(st.tas_kt, wind);
      const reqVs = ((startAlt - endAlt) / avail) * (gs / 60);
      const reqAng = radToDeg(Math.atan((startAlt - endAlt) / (avail * FT_PER_NM)));
      // Same idle simulation the stay-high planner used (250 kt until the slowdown,
      // level-offs, method B, decel segments, high-speed idle model).
      const endIas = "ceiling_end_ias" in nxt ? nxt.ceiling_end_ias : nxt.planned_ias_kias;
      const startIas = nxt.ceiling_start_ias;
      const ceilOpts = Object.assign({}, stayHighOpts, { startIas: startIas });
      const ceil = Math.min(idleStartAltitude(endAlt, endIas, avail, ceilOpts), +cruiseAlt);
      if (startAlt <= ceil + 25) continue; // idle-feasible: a path the planner can fly at idle
      const need = idleDescentDistance(startAlt, endAlt, endIas, ceilOpts);
      const idleVs = (Math.max(0, ceil - endAlt) / avail) * (gs / 60);
      const idleAng = radToDeg(Math.atan(Math.max(0, ceil - endAlt) / (avail * FT_PER_NM)));
      if (startAlt > ceil + 25) { // drag truly unavoidable
        const earlier = Math.max(0, need - avail);
        let msg = steepMessage(cur.name, nxt.name, startAlt, endAlt, avail, need, reqVs, idleVs, reqAng, idleAng, earlier);
        if (midAlt > (highEndAlt || 0) + 1) {
          msg += " A steeper path would push speed above " + highSpeedIas.toFixed(0) + " KIAS; speed brakes would be needed.";
        }
        const heldFrom = cur.held_level_from || nxt.held_level_from;
        const heldAlt = cur.held_level_alt != null ? cur.held_level_alt : nxt.held_level_alt;
        if (heldFrom && heldAlt != null) {
          msg +=
            " Level at " +
            heldAlt.toFixed(0) +
            " ft to " +
            (nxt.held_level_from ? nxt.name : cur.name) +
            " then descend.";
        } else if (i > 0 && Math.abs((cur.planned_alt_ft || 0) - (resolved[i - 1].planned_alt_ft || 0)) <= 50) {
          msg += " Level at " + cur.planned_alt_ft.toFixed(0) + " ft to " + cur.name + " then descend.";
        }
        flags.push(msg);
        cur.notes.push(msg);
      }
    }

    let outputWps = resolved.slice();
    const has10k = resolved.some((r) => r.planned_alt_ft != null && Math.abs(r.planned_alt_ft - 10000) <= 50);
    const crosses10k =
      resolved.some((r) => (r.planned_alt_ft || 0) > 10050) && resolved.some((r) => (r.planned_alt_ft == null ? 1e9 : r.planned_alt_ft) < 9950);
    if (
      !has10k &&
      (crosses10k ||
        (resolved.length &&
          (resolved[0].planned_alt_ft || 0) > 10050 &&
          (resolved[resolved.length - 1].planned_alt_ft || 0) <= 10000))
    ) {
      let insertAt = 0;
      for (let i = 0; i < resolved.length; i++) {
        if (resolved[i].planned_alt_ft != null && resolved[i].planned_alt_ft > 10000) insertAt = i + 1;
      }
      const prev = insertAt > 0 ? outputWps[insertAt - 1] : null;
      const nxt = insertAt < outputWps.length ? outputWps[insertAt] : null;
      let dist = matchDistanceByAltitude(10000);
      if (prev && nxt && prev.distance_from_tod_nm != null && nxt.distance_from_tod_nm != null) {
        const pa = prev.planned_alt_ft, na = nxt.planned_alt_ft;
        if (pa && na && pa !== na) {
          let frac = (pa - 10000) / (pa - na);
          frac = Math.min(Math.max(frac, 0), 1);
          dist = prev.distance_from_tod_nm + frac * (nxt.distance_from_tod_nm - prev.distance_from_tod_nm);
        }
      }
      if (dist == null) dist = anchorDist;
      let legTo10k = 0;
      if (prev && dist != null && prev.distance_from_tod_nm != null) {
        legTo10k = Math.max(0, dist - prev.distance_from_tod_nm);
      }
      if (nxt && dist != null && nxt.distance_from_tod_nm != null) {
        nxt.leg_distance_nm = Math.max(0, nxt.distance_from_tod_nm - dist);
      }
      const st = speedState(10000, {
        descent_mach: descentMach,
        high_speed_ias: highSpeedIas,
        ias_held: 250,
        isa_dev_c: isaDevC,
      });
      const gs = groundspeedKt(st.tas_kt, wind);
      outputWps.splice(insertAt, 0, {
        index: null,
        name: "10,000 ft",
        leg_distance_nm: round(legTo10k, 2),
        distance_from_start_nm: null,
        constraint_typ: "at",
        constraint_target_ft: 10000,
        constraint_min_ft: 10000,
        constraint_max_ft: 10000,
        speed_constraint: 250,
        planned_alt_ft: 10000,
        planned_ias_kias: 250,
        required_vs_fpm: vsForWaypoint(10000, 250),
        synthetic: true,
        is_builtin: true,
        notes: ["Built-in 250 KIAS / 10,000 ft constraint (already met if you were 250 at a higher restriction)."],
        tas_kt: round(st.tas_kt, 1),
        gs_kt: round(gs, 1),
        mach: round(st.mach, 3),
        distance_from_tod_nm: dist != null ? round(dist, 2) : null,
      });
    }

    annotateConstraintLegs(outputWps, {
      cruiseAlt,
      highEndAlt,
      idleAngle,
      vsCap,
      idle250,
      idle210,
      descentMach,
      highSpeedIas,
      isaDevC,
      wind,
      band: inputs.energy_idle_band != null ? +inputs.energy_idle_band : DEFAULT_ENERGY_IDLE_BAND,
      stayHighOpts,
    });

    let mbOffset = 0;
    if (method === "B" && methodBStart != null && alt250 != null) mbOffset = +methodBStart - +alt250;
    const fmsLegs = buildFmsLegs(outputWps, {
      idleAngle,
      vsCap,
      idle250,
      idle210,
      method,
      gateAlt: alt250 != null ? alt250 : 10000,
      methodBOffset: mbOffset,
      methodBVs: method === "B" ? methodBVs : null,
      methodAKtNm,
      decelBaseNm: inputs.decel_210_distance_nm != null ? +inputs.decel_210_distance_nm : 5,
      descentMach,
      highSpeedIas,
      isaDevC,
      wind,
      cruiseAlt,
      decelSegments,
    });
    const fmsLegsNotes = [
      "Along-track waypoints use the FMS 4200 format [IDENT/DIST] from AOM Vol 2 10.2.4 (e.g. CID/10, DSM/-11.5, ALO/+25/PDL). Negative distance is before the fix.",
      "Every entry is a hard 'at' altitude on the stay-high idle path, not the chart window or at/above notation. AOM Vol 2 2.7.1 Note 2: on FMS 4.2, a window is flown to the bottom; it may be replaced with a hard altitude for a more efficient profile.",
      "Speed and altitude are shown as 'KIAS / ALT' for reference. The AOM does not give the LEGS scratchpad keystrokes for a hard altitude or a combined speed/altitude entry, so that part is generic.",
      "Hard altitudes are rounded to the nearest 100 ft (the AOM does not state the FMS altitude entry resolution). Each STAR crossing is checked against the published restriction and flagged if it would fall outside.",
      "VNAV computes a path to each waypoint with an altitude constraint and can level at a flight-plan altitude, then start down at a planned location (AOM Vol 1 34-61). Prefer hard restrictions over entering a flight path angle. The FMS default descent angle is 4.0°; RNAV STARs are typically designed at 3.0° (AOM Vol 2 2.7.1). Descent angles, if used, are changed on the LEGS or ACT VNAV DESCENT page, not FMS DEFAULTS. Use of descent angles greater than 4.5° at high altitude may cause unintended acceleration (AOM Vol 2 4.5.2). The VNAV path is not temperature compensated; TEMP COMP is prohibited on FMS 4.2.",
    ];

    const capNote = capAppliedAtTod
      ? " Starting VS is capped at " + vsCap.toFixed(0) + " fpm (model would give " + uncappedTodVs.toFixed(0) + " fpm)."
      : " " + vsCap.toFixed(0) + " fpm is the maximum idle VS cap (not reached here).";

    const tod = {
      altitude_ft: cruiseAlt,
      required_vs_fpm: round(todVs, 0),
      vs_cap_fpm: vsCap,
      uncapped_vs_fpm: round(uncappedTodVs, 0),
      cap_applied: capAppliedAtTod,
      idle_path_angle_deg: idleAngleDeg,
      flight_path_angle_deg: round(todAngleDeg, 2),
      ias_kias: round(cruiseSt.ias_kias, 1),
      mach: round(cruiseSt.mach, 3),
      tas_kt: round(cruiseSt.tas_kt, 1),
      gs_kt: round(cruiseGs, 1),
      regime: cruiseSt.regime,
      distance_nm_to_high_speed_end: round(todToHighEnd, 2),
      distance_nm_to_last_waypoint: round(totalProfileNm, 2),
      anti_ice_advance_nm: round(antiIceNm, 2),
      idle_model: idleSummary,
      note:
        "Start down at idle on the " + hsLabel + " profile (idle VS " + todVs.toFixed(0) + " fpm at cruise, " +
        todAngleDeg.toFixed(2) + "° over the ground). VS at each step is TAS × tan(idle angle); it falls as TAS drops below the Mach/IAS crossover." +
        capNote +
        " Calibrated from 3 legs on 2026-11-23.",
    };
    if (resolved.length && resolved[0].distance_from_tod_nm != null) {
      tod.before_first_fix = { fix: resolved[0].name, distance_nm: round(resolved[0].distance_from_tod_nm, 2) };
    }

    return {
      example_data: !!inputs.example_data,
      example_note: inputs.example_data ? "FICTIONAL example STAR for testing only. Not a published procedure." : null,
      performance_source: PERF.source_label,
      weight: {
        class: rates.class,
        landing_weight_lb: inputs.landing_weight_lb != null ? inputs.landing_weight_lb : null,
        rate_source: rates.used,
        idle_250_kias_fpm: idle250,
        idle_210_kias_fpm: idle210,
      },
      atmosphere: {
        isa_dev_c: isaDevC,
        wind_component_kt: wind,
        wind_sign: "tailwind if positive, headwind if negative",
        descent_mach: descentMach,
        high_speed_ias_kias: highSpeedIas,
        idle_path_angle_deg: idleAngleDeg,
        idle_model: idleSummary,
        tod_vs_cap_fpm: vsCap,
        mach_ias_crossover_ft: round(xo, 0),
        step_ft: stepFt,
        energy_idle_band: inputs.energy_idle_band != null ? +inputs.energy_idle_band : DEFAULT_ENERGY_IDLE_BAND,
      },
      decel_method: method,
      tod,
      waypoints: outputWps.map((r) => ({
        name: r.name,
        leg_distance_nm: r.leg_distance_nm,
        target_crossing_altitude_ft: r.open ? null : r.planned_alt_ft,
        target_speed_kias: r.planned_ias_kias,
        required_vs_fpm: r.required_vs_fpm,
        tas_kt: r.tas_kt,
        gs_kt: r.gs_kt,
        mach: r.mach,
        distance_from_tod_nm: r.distance_from_tod_nm,
        altitude_constraint: { target_ft: r.constraint_target_ft, min_ft: r.constraint_min_ft, max_ft: r.constraint_max_ft },
        speed_constraint_kias: r.speed_constraint,
        notes: r.notes,
        synthetic: !!r.synthetic,
        is_builtin: !!r.is_builtin,
        leg_from: r.leg_from,
        leg_nm: r.leg_nm,
        leg_required_vs_fpm: r.leg_required_vs_fpm,
        leg_path_angle_deg: r.leg_path_angle_deg,
        leg_idle_vs_fpm: r.leg_idle_vs_fpm,
        leg_energy_state: r.leg_energy_state,
        leg_energy_label: r.leg_energy_label,
        leg_gs_kt: r.leg_gs_kt,
        idle_ceiling_ft: r.idle_ceiling_ft == null ? null : round(r.idle_ceiling_ft, 0),
        inbound_drag: !!r.inbound_drag,
        open: !!r.open,
        open_idle_estimate_ft: r.open && fieldElev != null && r.open_idle_estimate_ft != null ? Math.round(r.open_idle_estimate_ft) : null,
      })),
      deceleration_segments: decelSegments,
      profile_steps: assembled,
      flags: Array.from(new Set(flags)), // the stay-high pass runs twice; drop exact repeats
      fms_legs: fmsLegs,
      fms_legs_notes: fmsLegsNotes,
    };
  }

  function exampleStarInputs(weightClass, decelMethod) {
    return {
      waypoints: [
        { name: "RIVER", leg_distance_nm: 40, altitude_constraint: { typ: "at_or_below", altitude_ft: 24000 }, speed_constraint: null },
        { name: "CEDAR", leg_distance_nm: 36, altitude_constraint: { typ: "at", altitude_ft: 10000 }, speed_constraint: 250 },
        { name: "MAPLE", leg_distance_nm: 16, altitude_constraint: { typ: "window", min_ft: 5000, max_ft: 6000 }, speed_constraint: 210 },
        { name: "FAFXX", leg_distance_nm: 10, altitude_constraint: { typ: "at", altitude_ft: 2500 }, speed_constraint: 180 },
      ],
      cruise_altitude_ft: 35000,
      weight_class: weightClass || "medium",
      decel_method: decelMethod || "A",
      method_b_start_altitude_ft: 14000,
      wind_component_kt: 0,
      isa_dev_c: 0,
      descent_mach: DEFAULT_DESCENT_MACH,
      decel_210_distance_nm: DEFAULT_DECEL_210_NM,
      example_data: true,
    };
  }

  function exampleStarEntryWaypoints() {
    return [
      { name: "FAFXX", leg_distance_nm: 0, altitude_constraint: { typ: "at", altitude_ft: 2500 }, speed_constraint: 180 },
      { name: "MAPLE", leg_distance_nm: 10, altitude_constraint: { typ: "window", min_ft: 5000, max_ft: 6000 }, speed_constraint: 210 },
      { name: "CEDAR", leg_distance_nm: 16, altitude_constraint: { typ: "at", altitude_ft: 10000 }, speed_constraint: 250 },
      { name: "RIVER", leg_distance_nm: 36, altitude_constraint: { typ: "at_or_below", altitude_ft: 24000 }, speed_constraint: null },
    ];
  }

  function norak6EntryWaypoints() {
    return [
      { name: "IPSIV", leg_distance_nm: 0, altitude_constraint: { typ: "at_or_below", altitude_ft: 3000 }, speed_constraint: 210 },
      { name: "PESDA", leg_distance_nm: 12, altitude_constraint: { typ: "at_or_below", altitude_ft: 6000 }, speed_constraint: 230 },
      { name: "NORAK", leg_distance_nm: 21, altitude_constraint: { typ: "at_or_below", altitude_ft: 11000 }, speed_constraint: 250 },
    ];
  }

  function norak6Inputs(weightClass, decelMethod) {
    return {
      waypoints: entryToFlightOrder(norak6EntryWaypoints()),
      cruise_altitude_ft: 37000,
      weight_class: weightClass || "medium",
      decel_method: decelMethod || "B",
      method_b_start_altitude_ft: 4000,
      wind_component_kt: 0,
      isa_dev_c: 0,
      descent_mach: DEFAULT_DESCENT_MACH,
      decel_210_distance_nm: DEFAULT_DECEL_210_NM,
      example_data: false,
    };
  }

  function cuugr24lEntryWaypoints() {
    return [
      { name: "SKRUG", leg_distance_nm: 0, altitude_constraint: { typ: "at", altitude_ft: 8000 }, speed_constraint: 210 },
      { name: "ZUBBY", leg_distance_nm: 14, altitude_constraint: { typ: "none" }, speed_constraint: null },
      { name: "CUUGR", leg_distance_nm: 15, altitude_constraint: { typ: "window", min_ft: 10000, max_ft: 11000 }, speed_constraint: 250 },
      { name: "HHUNT", leg_distance_nm: 12, altitude_constraint: { typ: "window", min_ft: 12000, max_ft: 15000 }, speed_constraint: null },
      { name: "TPGUN", leg_distance_nm: 12, altitude_constraint: { typ: "window", min_ft: 16000, max_ft: 23000 }, speed_constraint: 280 },
    ];
  }

  function cuugr24lInputs(weightClass, decelMethod) {
    return {
      waypoints: entryToFlightOrder(cuugr24lEntryWaypoints()),
      cruise_altitude_ft: 37000,
      weight_class: weightClass || "medium",
      decel_method: decelMethod || "B",
      method_b_start_altitude_ft: 4000,
      wind_component_kt: 0,
      isa_dev_c: 0,
      descent_mach: DEFAULT_DESCENT_MACH,
      high_speed_ias_kias: 280,
      example_data: false,
    };
  }

  return {
    PERF,
    casToMachTas,
    machToCasTas,
    crossoverAltitudeFt,
    vsFromAngleGs,
    angleFromVsGs,
    resolveRates,
    plan,
    exampleStarInputs,
    exampleStarEntryWaypoints,
    entryToFlightOrder,
    idleVsForIas,
    methodBOffsetAndRate,
    norak6Inputs,
    norak6EntryWaypoints,
    cuugr24lInputs,
    cuugr24lEntryWaypoints,
    classifyEnergyState,
    DEFAULT_DESCENT_MACH,
    DEFAULT_ENERGY_IDLE_BAND,
    ENERGY_IDLE,
    ENERGY_POWER_ON,
    ENERGY_ADD_DRAG,
    ENERGY_STAY_HIGH,
    idleStartAltitude,
    idleDescentDistance,
    formatAlongTrackIdent,
    buildFmsLegs,
    DEFAULT_HIGH_SPEED_IAS,
    DEFAULT_IDLE_PATH_ANGLE_DEG,
    DEFAULT_TOD_VS_CAP_FPM,
    LEGACY_IDLE_PATH_ANGLE_DEG,
    DESCENT_IAS_MIN_KIAS,
    DESCENT_IAS_MAX_KIAS,
    IDLE_MACH_REF_VS_FPM,
    makeIdleModel,
    idleVsAtGs,
    weightClassForLb,
    WEIGHT_MIN_LB,
    WEIGHT_MAX_LB,
  };
});
