/**
 * CRJ900 cold-temperature altimeter corrections and FMS LEGS entry advice.
 * Source of every rule: Jazz COM Rev 31 §4.7.6, CRJ AOM Vol II (Rev 26–28) §2.7.3 8),
 * §2.7.9 5), §2.7.16 2), §4.5.2, §4.5.3, CRJ900 QRH OPERATIONS-1/-2, CRJ900 FCOM Vol 1 34-61-21.
 * Planning aid only — verify against the current COM / AOM / QRH.
 */
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.CRJColdTemp = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  /* ---------- QRH OPERATIONS-1 (Rev 19, 15 May 2025) correction table ---------- */
  const TABLE_TEMPS_C = [10, 0, -10, -20, -30, -40, -50];
  const TABLE_HEIGHTS_FT = [200, 300, 400, 500, 600, 700, 800, 900, 1000, 1500, 2000, 3000, 4000, 5000];
  // null = shaded (no value published) in the +10°C row below 1000 ft.
  const TABLE = [
    [null, null, null, null, null, null, null, null, 20, 30, 40, 60, 80, 100],
    [20, 20, 30, 30, 40, 40, 50, 50, 60, 90, 120, 170, 230, 290],
    [20, 30, 40, 50, 60, 70, 80, 90, 100, 150, 200, 290, 390, 490],
    [30, 50, 60, 70, 90, 100, 120, 130, 140, 210, 280, 420, 570, 710],
    [40, 60, 80, 100, 120, 140, 150, 170, 190, 280, 380, 570, 760, 950],
    [50, 80, 100, 120, 150, 170, 190, 220, 240, 360, 480, 720, 970, 1210],
    [60, 90, 120, 150, 180, 210, 240, 270, 300, 450, 590, 890, 1190, 1500],
  ];

  const L0_C_PER_FT = 0.0019812; // 0.0065 °C/m

  function roundUp10(x) {
    if (!(x > 0)) return 0;
    return Math.ceil(x / 10 - 1e-9) * 10;
  }

  /** Raw table cell; +10°C shaded cells fall back to the 0°C row (conservative). */
  function cell(ti, hi) {
    const v = TABLE[ti][hi];
    return v == null ? TABLE[1][hi] : v;
  }

  /** ICAO PANS-OPS (Doc 8168) approximate formula. Only used outside the QRH table. */
  function icaoCorrection(heightFt, tempC, sourceElevFt) {
    const hss = sourceElevFt || 0;
    const num = heightFt * (15 - tempC);
    const den = 273 + tempC - 0.5 * L0_C_PER_FT * (heightFt + hss);
    return num / den;
  }

  function bracket(arr, v, descending) {
    // returns [i, j, f] with arr[i] <= v <= arr[j] (by order) and fraction f
    for (let i = 0; i < arr.length - 1; i++) {
      const a = arr[i], b = arr[i + 1];
      const inside = descending ? v <= a && v >= b : v >= a && v <= b;
      if (inside) return [i, i + 1, (v - a) / (b - a)];
    }
    return null;
  }

  /**
   * Correction (ft, rounded up to next 10 ft) to add to a published altitude.
   * @param {number} heightFt height above the altimeter source (published alt − source elevation)
   * @param {number} tempC reported airport temperature
   * @param {object} opts { method: "interp"|"conservative", sourceElevFt }
   */
  function correction(heightFt, tempC, opts) {
    opts = opts || {};
    const method = opts.method === "conservative" ? "conservative" : "interp";
    const notes = [];
    if (!isFinite(heightFt) || !isFinite(tempC)) throw new Error("Height and temperature must be numbers");
    if (heightFt <= 0) return { correction_ft: 0, source: "none", notes: ["At or below the altimeter source — no correction."] };
    if (tempC > 10) return { correction_ft: 0, source: "none", notes: ["Warmer than +10°C — QRH table has no correction."] };

    let h = heightFt;
    if (h < 200) {
      h = 200;
      notes.push("Below 200 ft the QRH table has no column; the 200 ft value is used (conservative, manual silent).");
    }
    const outside = h > 5000 || tempC < -50;
    let tableVal = null;
    const hq = Math.min(h, 5000);
    const tq = Math.max(tempC, -50);

    if (method === "conservative") {
      // next colder row, next higher column
      let ti = TABLE_TEMPS_C.findIndex((x) => x <= tq);
      if (ti < 0) ti = TABLE_TEMPS_C.length - 1;
      let hi = TABLE_HEIGHTS_FT.findIndex((x) => x >= hq);
      if (hi < 0) hi = TABLE_HEIGHTS_FT.length - 1;
      if (TABLE[ti][hi] == null) notes.push("+10°C row is shaded below 1,000 ft; 0°C value used.");
      tableVal = cell(ti, hi);
    } else {
      const tb = bracket(TABLE_TEMPS_C, tq, true);
      const hb = bracket(TABLE_HEIGHTS_FT, hq, false);
      const [t0, t1, ft] = tb;
      const [h0, h1, fh] = hb;
      if (t0 === 0 && hq < 1000) notes.push("+10°C row is shaded below 1,000 ft; 0°C value used for that row (conservative).");
      const a = cell(t0, h0) + (cell(t0, h1) - cell(t0, h0)) * fh;
      const b = cell(t1, h0) + (cell(t1, h1) - cell(t1, h0)) * fh;
      tableVal = a + (b - a) * ft;
    }

    let val = tableVal;
    let source = method === "conservative" ? "QRH table (next colder row / higher column)" : "QRH table (interpolated)";
    if (outside) {
      const f = icaoCorrection(heightFt, tempC, opts.sourceElevFt);
      val = Math.max(f, tableVal);
      source = "ICAO PANS-OPS formula (outside QRH table)";
      notes.push("Outside the QRH table (above 5,000 ft or colder than −50°C). Jazz manuals give no method here; ICAO Doc 8168 formula used, never less than the table edge value.");
    }
    return { correction_ft: roundUp10(val), source, notes };
  }

  /* ---------- Options ---------- */
  const FMS_OPTIONS = [
    { id: "fms99", label: "FMS 99 software", sub: "Fins 701–716 · NAV DATA J4 AMERICA" },
    { id: "fms42", label: "FMS 4.2 software", sub: "Fins 717–730 (CRJ900 NG) · NAV DATA J8-AMERICA" },
    { id: "unknown", label: "Not sure", sub: "Show both" },
  ];

  const APPROACH_OPTIONS = [
    { id: "ils", label: "ILS" },
    { id: "loc", label: "LOC" },
    { id: "lda", label: "LDA" },
    { id: "vor", label: "VOR" },
    { id: "ndb", label: "NDB" },
    { id: "lnav", label: "RNAV – LNAV" },
    { id: "lnavvnav", label: "RNAV – LNAV/VNAV" },
    { id: "lpv", label: "RNAV – LPV" },
    { id: "visual", label: "Visual / visual manoeuvring" },
    { id: "star", label: "STAR legs" },
    { id: "sid", label: "SID legs" },
  ];

  // QRH OPERATIONS-2 / AOM Vol II 2.7.3 8): "not required to be entered ... but may be entered as follows"
  const QRH_LEGS = {
    fms99: { ils: "YES", loc: "YES", lda: "YES", vor: "YES", lnav: "YES", lpv: "N/A", lnavvnav: "N/A", missed: "YES" },
    fms42: { ils: "YES", loc: "YES", lda: "YES", vor: "YES", lnav: "NO", lpv: "NO", lnavvnav: "NO", missed: "NO" },
  };
  const CITE_TABLE = "QRH OPERATIONS-2 (Rev 20); AOM Vol II 2.7.3 8) p. 2.7-7";

  /** LEGS entry answer for the approach (non-missed) altitudes. */
  function legsForApproach(fms, appr) {
    if (fms === "unknown") {
      const a = legsForApproach("fms99", appr), b = legsForApproach("fms42", appr);
      if (a.answer === b.answer) return Object.assign({}, a, { reason: a.reason + " (same for both FMS loads)" });
      return {
        answer: "CHECK",
        reason: "Answer depends on the FMS: FMS 99 " + a.answer + ", FMS 4.2 " + b.answer + ". Check the fin number (701–716 = FMS 99, 717–730 = FMS 4.2) or the STATUS page NAV DATA name (J4 AMERICA = FMS 99, J8-AMERICA = FMS 4.2).",
        cite: "QRH OPERATIONS-2; AOM Vol II 1.1.2, 4.5.2 4), 10.2.1",
        silent: false,
      };
    }
    const row = QRH_LEGS[fms];
    if (row && row[appr] != null) {
      const ans = row[appr];
      let reason;
      if (ans === "YES") reason = "May be entered as hard altitudes on LEGS (optional — not required).";
      else if (ans === "NO") reason = "Do not enter corrected altitudes on LEGS for this approach with FMS 4.2.";
      else reason = "Approach type not available with FMS 99 software.";
      return { answer: ans, reason, cite: CITE_TABLE, silent: false };
    }
    if (appr === "sid") {
      if (fms === "fms42") return {
        answer: "NO",
        reason: "Corrected 'climb-to' altitudes (SID or missed approach) shall not be programmed on LEGS; changes to departure altitudes are prohibited with FMS 4.2.",
        cite: "AOM Vol II 2.7.3 8) Caution p. 2.7-8; AOM Vol II 4.5.2 p. 4.5-2", silent: false,
      };
      return { answer: "CHECK", reason: "The manuals do not say whether corrected SID altitudes may be entered on LEGS with FMS 99.", cite: "Manual silent", silent: true };
    }
    const silentText = {
      ndb: "NDB is not listed in the QRH LEGS table.",
      visual: "Visual approaches are not listed in the QRH LEGS table. Compensation applies only when the approach is planned in IMC or night VMC (QRH OPERATIONS-1).",
      star: "STAR legs are not covered by the QRH LEGS table. COM 4.7.6.6: the IAF is not compensated if it is a STAR interface waypoint.",
    };
    return { answer: "CHECK", reason: (silentText[appr] || "Not covered by the QRH LEGS table.") + " Check the manual.", cite: "Manual silent", silent: true };
  }

  function legsForMissed(fms) {
    if (fms === "unknown") {
      return { answer: "CHECK", reason: "FMS 99: YES. FMS 4.2: NO. Identify the fin / FMS load.", cite: CITE_TABLE, silent: false };
    }
    if (fms === "fms42") return {
      answer: "NO",
      reason: "Missed approach: NO with FMS 4.2. Corrected climb-to altitudes shall not be programmed (FMS may not fly the procedure as published) and missed-approach altitude changes are prohibited.",
      cite: CITE_TABLE + "; AOM Vol II 2.7.3 8) Caution p. 2.7-8; 4.5.2 p. 4.5-2", silent: false,
    };
    return { answer: "YES", reason: "Missed approach altitudes may be entered on LEGS with FMS 99 (optional).", cite: CITE_TABLE, silent: false };
  }

  /* ---------- When compensation is required ---------- */
  /**
   * @param {object} p {country:"CA"|"US", tempC, ctaTempC (US, null if none published),
   *   imcOrNight:boolean, daMdaAaeFt (DA/MDA height above aerodrome, or null), minimaIsDH:boolean,
   *   radar:boolean}
   */
  function requirements(p) {
    const t = p.tempC;
    const out = {};
    const minimaHigh = p.daMdaAaeFt != null && p.daMdaAaeFt >= 1000;
    // Non-approach minimum IFR altitudes (MSA / TAA / 100 NM safe) — COM 4.7.6.2
    if (p.radar) out.msa = { required: false, reason: "Under radar control — MSA/TAA/100 NM safe altitude need not be compensated.", cite: "COM 4.7.6.2 p. 4.7-6" };
    else out.msa = t <= 0
      ? { required: true, reason: "Airport temperature at or below 0°C.", cite: "COM 4.7.6.2 p. 4.7-6" }
      : { required: false, reason: "Above 0°C.", cite: "COM 4.7.6.2 p. 4.7-6" };

    if (!p.imcOrNight) {
      const r = { required: false, reason: "Day VMC — compensation is required when the approach is planned in IMC or night VMC.", cite: "QRH OPERATIONS-1; AOM Vol II 2.7.3 8)" };
      out.approach = r; out.minima = Object.assign({}, r); out.missed = Object.assign({}, r);
      return out;
    }

    // Approach minima (DA/MDA) — COM 4.7.6.3
    if (p.minimaIsDH) out.minima = { required: false, reason: "Radio-altitude DH — no compensation.", cite: "COM 4.7.6.3 p. 4.7-6" };
    else if (t <= 0) out.minima = { required: true, reason: "At or below 0°C.", cite: "COM 4.7.6.3 p. 4.7-6" };
    else if (t <= 10 && minimaHigh) out.minima = { required: true, reason: "+10°C or below with DA/MDA ≥ 1,000 ft AAE.", cite: "COM 4.7.6.3 p. 4.7-6" };
    else if (t <= 10 && p.daMdaAaeFt == null) out.minima = { required: null, reason: "Enter the DA/MDA — required at +10°C or below if it is ≥ 1,000 ft AAE.", cite: "COM 4.7.6.3" };
    else out.minima = { required: false, reason: t > 10 ? "Warmer than +10°C." : "Above 0°C and DA/MDA below 1,000 ft AAE.", cite: "COM 4.7.6.3 p. 4.7-6" };

    // Approach and missed approach altitudes — COM 4.7.6.6–4.7.6.9
    if (p.country === "US") {
      if (p.ctaTempC == null) out.approach = { required: false, reason: "USA, no Cold Temperature Airport (CTA) temperature in the Jeppesen briefing strip — not required, nor expected by ATC.", cite: "COM 4.7.6.8 p. 4.7-8" };
      else if (t <= p.ctaTempC) out.approach = { required: true, reason: "USA, at or below the published CTA temperature (" + p.ctaTempC + "°C).", cite: "COM 4.7.6.8 p. 4.7-8" };
      else out.approach = { required: false, reason: "USA, warmer than the published CTA temperature (" + p.ctaTempC + "°C).", cite: "COM 4.7.6.8 p. 4.7-8" };
    } else {
      if (t <= 0) out.approach = { required: true, reason: "Canada, at or below 0°C.", cite: "COM 4.7.6.7 p. 4.7-7; 4.7.6.9" };
      else if (t <= 10 && minimaHigh) out.approach = { required: null, reason: "Canada, +10°C or below with DA/MDA ≥ 1,000 ft AAE. The COM table puts approach altitudes and minima on one row with this trigger, but 4.7.6.3 names only DA/MDA. Check the manual; corrections are shown.", cite: "COM 4.7.6.3, 4.7.6.7, 4.7.6.9 — wording ambiguous" };
      else out.approach = { required: false, reason: t > 10 ? "Canada, warmer than +10°C." : "Canada, above 0°C (DA/MDA below 1,000 ft AAE).", cite: "COM 4.7.6.7 p. 4.7-7" };
    }
    out.missed = Object.assign({}, out.approach, { reason: out.approach.reason + " (Missed approach final level-off altitude is an approach altitude, COM 4.7.6.6.)" });
    return out;
  }

  /* ---------- Altitude kinds offered on the page ---------- */
  const ALT_KINDS = [
    { id: "iaf", label: "IAF", group: "approach" },
    { id: "pt", label: "Procedure turn", group: "approach" },
    { id: "if", label: "IF", group: "approach" },
    { id: "stepdown", label: "Step-down fix", group: "approach" },
    { id: "faf", label: "FAF", group: "approach" },
    { id: "cda", label: "SCDA / CDA altitude", group: "approach" },
    { id: "daMda", label: "DA / MDA", group: "minima" },
    { id: "missed", label: "Missed approach level-off", group: "missed" },
    { id: "maHold", label: "Missed approach hold", group: "missed" },
    { id: "msa", label: "MSA / TAA / 100 NM safe", group: "msa" },
    { id: "other", label: "Other", group: "approach" },
  ];
  const KIND = Object.fromEntries(ALT_KINDS.map((k) => [k.id, k]));

  function isRnavFinalVnav(appr) { return appr === "lnavvnav"; }

  /**
   * Full evaluation for the page.
   * input: {fms, approach, country, tempC, elevFt, sourceElevFt?, ctaTempC?, imcOrNight, radar,
   *   minimaIsDH, scda, iafIsStarInterface, atcAssigned, tlimLowC?, tlimHighC?, method,
   *   altitudes: [{kind, name, altFt}] }
   */
  function evaluate(input) {
    const errs = [];
    const t = Number(input.tempC);
    const elev = Number(input.elevFt);
    if (!isFinite(t) || t < -70 || t > 60) errs.push("Enter the airport temperature in °C (−70 to +60).");
    if (!isFinite(elev) || elev < -1500 || elev > 15000) errs.push("Enter the aerodrome elevation in ft.");
    if (errs.length) return { ok: false, errors: errs };
    const srcElev = input.sourceElevFt != null && input.sourceElevFt !== "" && isFinite(Number(input.sourceElevFt)) ? Number(input.sourceElevFt) : elev;
    const fms = input.fms || "unknown";
    const appr = input.approach || "ils";

    const alts = (input.altitudes || []).filter((a) => a && a.altFt !== "" && a.altFt != null && isFinite(Number(a.altFt)));
    const daRow = alts.find((a) => a.kind === "daMda");
    const daMdaAaeFt = daRow ? Number(daRow.altFt) - elev : null;

    const req = requirements({
      country: input.country === "US" ? "US" : "CA",
      tempC: t,
      ctaTempC: input.ctaTempC === "" || input.ctaTempC == null || !isFinite(Number(input.ctaTempC)) ? null : Number(input.ctaTempC),
      imcOrNight: input.imcOrNight !== false,
      daMdaAaeFt,
      minimaIsDH: !!input.minimaIsDH,
      radar: !!input.radar,
    });
    if (input.scda && t <= 0 && input.imcOrNight !== false) {
      req.approach = { required: true, reason: "SCDA at or below 0°C surface temperature — corrections applied to all procedural altitudes, including the CDA altitude (VS is not corrected).", cite: "AOM Vol II 4.5.3 p. 4.5-10; COM 4.7.6.5 p. 4.7-6" };
      req.missed = Object.assign({}, req.approach);
    }

    const approachLegs = legsForApproach(fms, appr);
    const missedLegs = legsForMissed(fms);

    const rows = alts.map((a) => {
      const kind = KIND[a.kind] || KIND.other;
      const alt = Number(a.altFt);
      const height = alt - srcElev;
      const c = correction(height, t, { method: input.method, sourceElevFt: srcElev });
      let r = req[kind.group] || req.approach;
      const notes = c.notes.slice();
      if (kind.id === "iaf" && input.iafIsStarInterface) {
        r = { required: false, reason: "IAF is a STAR interface waypoint — not compensated.", cite: "COM 4.7.6.6 p. 4.7-7" };
      }
      if (kind.id === "pt") notes.push("COM 4.7.6.6 does not list procedure-turn altitude separately; treated as an approach altitude (it is usually the IAF/IF altitude). Verify.");
      if (kind.id === "cda" && !input.scda) notes.push("CDA altitude correction is specified for VS SCDA approaches (COM 4.7.6.5).");
      if (input.atcAssigned && kind.group !== "minima" && kind.group !== "msa") {
        r = { required: false, reason: "Superseded by an ATC-assigned altitude — an accepted ATC altitude is not adjusted.", cite: "COM 4.7.6.1, 4.7.6.6 note" };
      }
      let legs;
      if (kind.group === "minima") legs = { answer: "N/A", reason: "DA/MDA is set on the minimums, not a LEGS altitude. The manuals say nothing about putting it on LEGS.", cite: "Manual silent on LEGS entry for minima", silent: true };
      else if (kind.group === "msa") legs = { answer: "N/A", reason: "Not a LEGS altitude.", cite: "—", silent: false };
      else if (kind.group === "missed") legs = missedLegs;
      else legs = approachLegs;
      if (r.required === false && legs.answer === "YES") legs = Object.assign({}, legs, { reason: "Allowed, but no correction is required here. " + legs.reason });
      const corrected = r.required === false ? alt : alt + c.correction_ft;
      return {
        kind: kind.id, kindLabel: kind.label, name: a.name || kind.label,
        published_ft: alt, height_above_source_ft: height,
        correction_ft: c.correction_ft, corrected_ft: alt + c.correction_ft,
        use_ft: corrected, source: c.source, required: r.required, required_reason: r.reason, required_cite: r.cite,
        legs, notes,
      };
    });

    // Overall LEGS answer for the approach and conditions
    const conditions = [];
    const add = (text, cite) => conditions.push({ text, cite });
    add("Temperature-corrected altitudes are not required on the FMS LEGS page; where allowed they may be entered (as hard altitudes).", CITE_TABLE);
    add("Round corrections up to the next 10 ft. Use the destination aerodrome elevation as the altimeter-source elevation unless otherwise specified.", "COM 4.7.6 p. 4.7-5");
    add("Correcting a published mandatory altitude or missed approach holding altitude: advise ATC of the corrected altitude before crossing the fix.", "COM 4.7.6 p. 4.7-5");
    add("Do not correct an ATC-assigned altitude you have accepted; radar vectoring altitudes are already corrected. You may refuse an assigned altitude for obstacle clearance.", "COM 4.7.6.1 p. 4.7-6");
    add("Priority: 1) automatic FMS compensation (per aircraft capability), 2) QRH charts. If a published waypoint is not in the database-coded procedure, do not insert it into the FMS for compensation.", "COM 4.7.6.10 p. 4.7-9");
    add("The FMS VNAV is not temperature compensated; FMS altitudes and V-paths are not corrected for non-ISA and the path may be significantly below the intended path in cold temperatures.", "AOM Vol II 4.5.2 pp. 4.5-2/-3");
    if (fms !== "fms99") add("FMS 4.2 (CRJ900 NG): use of the TEMP COMP function is prohibited.", "AOM Vol II 4.5.2 p. 4.5-2; 2.7.16 2) Note 3; QRH OPERATIONS-2");
    if (fms !== "fms42") add("FMS 99: COM 4.7.6.4 says use TEMP COMP for RNAV approaches \u201cif available\u201d. The manuals do not say whether FMS 99 has it. Check the manual.", "COM 4.7.6.4 — manual silent for FMS 99");
    if (fms !== "fms99") add("FMS 4.2: changes to altitudes in departure and missed approach procedures are prohibited; corrected climb-to altitudes shall not be programmed on LEGS.", "AOM Vol II 4.5.2 p. 4.5-2; 2.7.3 8) Caution p. 2.7-8");
    if (["ils", "loc", "lda"].includes(appr)) add("ILS/LOC/LDA via the FMS are advisory only; VOR/LOC is the primary NAV source.", "AOM Vol II 4.5.2 p. 4.5-1");
    if (t <= 0 && input.imcOrNight !== false && ["lnav", "vor", "ndb", "loc", "lda"].includes(appr))
      add("ADVISORY VNAV (snowflake) display is prohibited in IMC or night VMC when the arrival airport surface temperature is 0°C or below.", "AOM Vol II 4.5.2 3) p. 4.5-3");
    if (appr === "lpv") add("LPV vertical guidance is SBAS-geometric and not affected by temperature; only the minimums need compensation.", "FCOM Vol 1 34-61-21");
    let tlim = null;
    if (isRnavFinalVnav(appr)) {
      add("LNAV/VNAV with BARO-VNAV is prohibited if corrected altitudes are entered on LEGS for the final approach segment (FAF and step-downs). Changes to final-segment altitudes are prohibited.", "AOM Vol II 2.7.16 2) Note 4 p. 2.7-39; 4.5.2 3) p. 4.5-4");
      add("If a TLIM is published, LNAV/VNAV minima using BARO-VNAV may be flown only with the reported temperature inside the TLIM range. Outside TLIM they can't be used unless automatic temperature correction is applied, and TEMP COMP is prohibited on FMS 4.2.", "AOM Vol II 2.7.3 8) Note p. 2.7-8; 4.5.2 3) p. 4.5-4; COM 4.7.6.9 note");
      const lo = input.tlimLowC === "" || input.tlimLowC == null ? null : Number(input.tlimLowC);
      const hi = input.tlimHighC === "" || input.tlimHighC == null ? null : Number(input.tlimHighC);
      if (lo != null && isFinite(lo)) {
        const inside = t >= lo && (hi == null || !isFinite(hi) || t <= hi);
        tlim = { low: lo, high: isFinite(hi) ? hi : null, inside };
      }
      if (fms === "fms99") add("FMS 99 aircraft are not approved for LNAV/VNAV (table shows N/A). APV approaches are approved for CRJ900 NG with FMS 4.2.", "QRH OPERATIONS-2; AOM Vol II 2.7.16 2)");
    }
    if (appr === "lpv" && fms === "fms99") add("FMS 99 aircraft: LPV is N/A in the QRH table.", "QRH OPERATIONS-2");
    if (input.scda) add("VS SCDA: apply compensation to altitudes including the CDA altitude; no compensation is applied to the vertical speed. The FMS does not correct altitude for low temperature.", "COM 4.7.6.5 p. 4.7-6; AOM Vol II 4.5.3 p. 4.5-10");
    add("US Cold Temperature Airports: the CTA temperature is in the Jeppesen briefing strip, in addition to any Baro-VNAV TLIM.", "COM 4.7.6.8 p. 4.7-8");

    const anyRequired = rows.some((r) => r.required === true);
    return {
      ok: true,
      fms, approach: appr, tempC: t, elevFt: elev, sourceElevFt: srcElev,
      requirements: req, approachLegs, missedLegs, tlim, rows, conditions, anyRequired,
    };
  }

  return {
    TABLE, TABLE_TEMPS_C, TABLE_HEIGHTS_FT, roundUp10, correction, icaoCorrection,
    FMS_OPTIONS, APPROACH_OPTIONS, ALT_KINDS, QRH_LEGS,
    legsForApproach, legsForMissed, requirements, evaluate,
  };
});
