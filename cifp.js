/* FAA CIFP STAR loader and path assembler for the CRJ descent planner. */
(function (global) {
  const VECTOR_LEGS = new Set(["FM", "VM"]);
  const ARC_LEGS = new Set(["AF", "RF"]);
  const HEADING_LEGS = new Set(["VA", "VI", "VD", "VR", "CA", "CD", "CI", "CR", "FA", "FC", "FD"]);

  function gcNm(lat1, lon1, lat2, lon2) {
    const r1 = (lat1 * Math.PI) / 180;
    const r2 = (lat2 * Math.PI) / 180;
    const dlat = r2 - r1;
    const dlon = ((lon2 - lon1) * Math.PI) / 180;
    const a = Math.sin(dlat / 2) ** 2 + Math.cos(r1) * Math.cos(r2) * Math.sin(dlon / 2) ** 2;
    const ang = 2 * Math.asin(Math.min(1, Math.sqrt(a)));
    return ang * (180 / Math.PI) * 60;
  }

  function formatNm(dist) {
    if (dist == null || Number.isNaN(dist)) return "";
    const x = Math.round((Number(dist) + 1e-12) * 10) / 10;
    if (Math.abs(x - Math.round(x)) < 1e-9) return String(Math.round(x));
    return x.toFixed(1);
  }

  function starTitle(ident) {
    const m = String(ident || "").trim().toUpperCase().match(/^([A-Z]{3,5})(\d{1,2}[A-Z]?)$/);
    return m ? `${m[1]} ${m[2]}` : String(ident || "").trim().toUpperCase();
  }

  function rwyTitle(ident) {
    const id = String(ident || "").trim().toUpperCase();
    const m = id.match(/^RW0?(\d{1,2})([LCRB]?)$/);
    if (m) {
      const num = String(parseInt(m[1], 10));
      return m[2] ? `RWY ${num}${m[2]}` : `RWY ${num}`;
    }
    return id || "Common";
  }

  function expandFix(f) {
    const out = {
      name: f.n || f.name || "",
      leg: f.leg || "TF",
      typ: f.typ || "none",
    };
    for (const k of ["alt", "min", "max", "spd", "lat", "lon", "flags", "d", "trk"]) {
      if (f[k] != null) out[k] = f[k];
    }
    return out;
  }

  function expandStar(compact) {
    const tr = compact.tr || compact.transitions || {};
    const rw = compact.rw || compact.runways || {};
    const cm = compact.cm != null ? compact.cm : compact.common || [];
    const out = {
      title: compact.title || "",
      transitions: Object.fromEntries(Object.entries(tr).map(([k, v]) => [k, v.map(expandFix)])),
      common: cm.map(expandFix),
      runways: Object.fromEntries(Object.entries(rw).map(([k, v]) => [k, v.map(expandFix)])),
    };
    if (compact.flags) out.flags = compact.flags;
    if (compact.chart_extracted) out.chart_extracted = true;
    if (compact.plates) out.plates = compact.plates;
    if (compact.cycle) out.cycle = compact.cycle;
    return out;
  }

  function commonStartOptions(star) {
    const common = star.common || [];
    if (!common.length) return [];
    const start = (common[0].name || "").trim();
    if (start && !Object.prototype.hasOwnProperty.call(star.transitions || {}, start)) return [start];
    return [];
  }

  function mergeFix(a, b) {
    const out = Object.assign({}, a);
    if ((out.typ || "none") === "none" && (b.typ || "none") !== "none") {
      for (const k of ["typ", "alt", "min", "max"]) {
        if (k in b) out[k] = b[k];
        else if (k !== "typ") delete out[k];
      }
      out.typ = b.typ || out.typ;
    }
    if (out.spd == null && b.spd != null) out.spd = b.spd;
    if (out.lat == null && b.lat != null) {
      out.lat = b.lat;
      out.lon = b.lon;
    }
    if (!out.leg || out.leg === "IF") {
      if (b.leg && !VECTOR_LEGS.has(b.leg)) out.leg = b.leg;
    }
    return out;
  }

  function appendSegment(path, flags, segment) {
    for (const fix of segment || []) {
      const leg = String(fix.leg || "").toUpperCase();
      const name = String(fix.name || "").trim();
      if (VECTOR_LEGS.has(leg)) {
        const last = path.length ? path[path.length - 1].name : name || "vectors";
        flags.push(`Vectors (${leg}) after ${last}; end of STAR is ${last}.`);
        return false;
      }
      if (!name) {
        flags.push(`Skipped ${leg || "unnamed"} heading/intercept leg.`);
        continue;
      }
      if (HEADING_LEGS.has(leg)) {
        flags.push(`Skipped ${leg} heading/intercept leg at ${name}.`);
      }
      const item = Object.assign({}, fix);
      const extra = [];
      if (ARC_LEGS.has(leg)) extra.push(`${leg} arc approximated as great-circle distance.`);
      if (item.lat == null) extra.push(`No coordinates for ${name}; enter the leg distance from the chart.`);
      if (extra.length) item.flags = extra;
      if (path.length && path[path.length - 1].name === name) {
        const merged = mergeFix(path[path.length - 1], item);
        if (extra.length) merged.flags = Array.from(new Set([].concat(merged.flags || [], extra)));
        path[path.length - 1] = merged;
        continue;
      }
      path.push(item);
    }
    return true;
  }

  function addDistances(path, flags) {
    const out = [];
    for (let i = 0; i < path.length; i++) {
      const item = Object.assign({}, path[i]);
      if (i === 0) {
        item.dist_nm = 0;
      } else {
        const prev = out[i - 1];
        if (item.d != null && item.d !== "") {
          item.dist_nm = Number(item.d);
          if (item.lat != null && prev.lat != null) {
            const gc = gcNm(prev.lat, prev.lon, item.lat, item.lon);
            if (Math.abs(gc - Number(item.d)) > 1.0) {
              flags.push(`${prev.name}–${item.name}: charted ${formatNm(item.d)} NM, great-circle ${formatNm(gc)} NM.`);
            }
          }
        } else if (item.lat != null && prev.lat != null) {
          const dist = gcNm(prev.lat, prev.lon, item.lat, item.lon);
          item.dist_nm = Math.round(dist * 100) / 100;
          if (ARC_LEGS.has(prev.leg) || ARC_LEGS.has(item.leg)) {
            flags.push(`${item.name}: distance ${formatNm(dist)} NM is a great-circle approximation of an arc.`);
          }
        } else {
          item.dist_nm = null;
          flags.push(`No distance computed for ${prev.name}–${item.name} (missing coordinates).`);
        }
      }
      out.push(item);
    }
    return out;
  }

  function entryWp(fix) {
    const typ = fix.typ || "none";
    return {
      name: fix.name || "",
      leg: "",
      typ,
      alt: typ === "window" || typ === "none" || fix.alt == null ? "" : String(fix.alt),
      min: typ !== "window" || fix.min == null ? "" : String(fix.min),
      max: typ !== "window" || fix.max == null ? "" : String(fix.max),
      spd: fix.spd == null ? "" : String(fix.spd),
    };
  }

  function flyingToEntry(path) {
    if (!path.length) return [];
    const n = path.length;
    return path.slice().reverse().map((fix, i) => {
      const wp = entryWp(fix);
      if (i === 0) wp.leg = "";
      else wp.leg = formatNm(path[n - i].dist_nm);
      return wp;
    });
  }

  function assembleArrival(stars, airport, procedure, transition, runway) {
    airport = String(airport || "").trim().toUpperCase();
    procedure = String(procedure || "").trim().toUpperCase();
    transition = String(transition || "").trim().toUpperCase() || null;
    runway = String(runway || "").trim().toUpperCase() || null;
    const apt = stars[airport];
    if (!apt) throw new Error(`${airport} has no FAA STARs in this CIFP cycle`);
    const star = apt[procedure];
    if (!star) throw new Error(`${airport} has no STAR ${procedure} in this CIFP cycle`);

    const flags = [];
    const path = [];
    const enroute = star.transitions || {};
    const common = star.common || [];
    const runways = star.runways || {};

    if (transition && enroute[transition]) {
      if (!appendSegment(path, flags, enroute[transition])) {
        return finalize(airport, procedure, star, transition, runway, path, flags);
      }
    } else if (transition && common.length && common[0].name === transition) {
      // Start at the first common-route fix (e.g. TPGUN on CUUGR 2).
    } else if (transition && transition !== "NONE" && !enroute[transition]) {
      flags.push(`Unknown transition ${transition}; starting at the common route.`);
    }

    if (common.length) {
      if (!appendSegment(path, flags, common)) {
        return finalize(airport, procedure, star, transition, runway, path, flags);
      }
    }

    if (runway && runways[runway]) {
      if (!appendSegment(path, flags, runways[runway])) {
        return finalize(airport, procedure, star, transition, runway, path, flags);
      }
    } else if (runway && !runways[runway]) {
      flags.push(`Unknown runway transition ${runway}.`);
    } else if (!runway && Object.keys(runways).length === 1) {
      const only = Object.keys(runways)[0];
      if (!appendSegment(path, flags, runways[only])) {
        return finalize(airport, procedure, star, transition, only, path, flags);
      }
      runway = only;
    }
    return finalize(airport, procedure, star, transition, runway, path, flags);
  }

  function finalize(airport, procedure, star, transition, runway, path, flags) {
    path = addDistances(path, flags);
    for (const msg of star.flags || []) {
      if (!flags.includes(msg)) flags.push(msg);
    }
    return {
      airport,
      procedure,
      title: star.title || starTitle(procedure),
      transition,
      runway,
      fixes: path,
      entry: flyingToEntry(path),
      flags,
      end_of_star: path.length ? path[path.length - 1].name : null,
    };
  }

  function cycleIsExpired(meta, now) {
    if (!meta || !meta.expires_utc) return false;
    const exp = Date.parse(meta.expires_utc);
    if (Number.isNaN(exp)) return false;
    return (now || Date.now()) >= exp;
  }

  function formatCycleDate(iso) {
    if (!iso) return "";
    const m = String(iso).match(/^(\d{4})-(\d{2})-(\d{2})/);
    if (!m) return iso;
    const months = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
    return `${parseInt(m[3], 10)} ${months[parseInt(m[2], 10) - 1]} ${m[1]}`;
  }

  async function fetchJson(url) {
    const resp = await fetch(url, { cache: "no-store" });
    if (!resp.ok) throw new Error(`Failed to load ${url} (${resp.status})`);
    return resp.json();
  }

  async function fetchGzipJson(url) {
    const resp = await fetch(url, { cache: "no-store" });
    if (!resp.ok) throw new Error(`Failed to load ${url} (${resp.status})`);
    if (typeof DecompressionStream !== "function" || !resp.body) {
      throw new Error("gzip is not supported in this browser");
    }
    const stream = resp.body.pipeThrough(new DecompressionStream("gzip"));
    const text = await new Response(stream).text();
    return JSON.parse(text);
  }

  function expandBundleAirports(airports) {
    const out = {};
    for (const [apt, procs] of Object.entries(airports || {})) {
      out[apt] = {};
      for (const [proc, star] of Object.entries(procs || {})) out[apt][proc] = expandStar(star);
    }
    return out;
  }

  async function loadBundle(base) {
    const root = base || "cifp";
    let raw = null;
    try {
      raw = await fetchGzipJson(`${root}/stars.json.gz`);
    } catch (err) {
      raw = await fetchJson(`${root}/stars.json`);
    }
    const meta = {
      cycle: raw.cycle,
      cycle_file: raw.cycle_file,
      volume: raw.volume,
      effective: raw.effective,
      expires: raw.expires,
      effective_utc: raw.effective_utc,
      expires_utc: raw.expires_utc,
      source: raw.source,
    };
    return { meta, stars: expandBundleAirports(raw.airports || {}) };
  }

  function listAirports(stars) {
    return Object.keys(stars || {}).sort();
  }

  function listStars(stars, airport) {
    const apt = stars[String(airport || "").toUpperCase()] || {};
    return Object.keys(apt)
      .sort()
      .map((id) => ({ id, title: apt[id].title || starTitle(id) }));
  }

  function listTransitions(star) {
    if (!star) return [];
    const enroute = Object.keys(star.transitions || {}).sort();
    const start = commonStartOptions(star);
    const seen = new Set();
    const out = [];
    for (const id of start.concat(enroute)) {
      if (!id || seen.has(id)) continue;
      seen.add(id);
      const isCommon = start.includes(id);
      out.push({ id, title: isCommon ? `${id} (start of arrival)` : id, common: isCommon });
    }
    return out;
  }

  function listRunways(star) {
    if (!star) return [];
    return Object.keys(star.runways || {})
      .sort()
      .map((id) => ({ id, title: rwyTitle(id) }));
  }


  async function loadCanadaManifest(base) {
    const root = base || "canada";
    return fetchJson(root + "/manifest.json");
  }

  async function loadCanadaBundle(base) {
    const root = base || "canada";
    const [manifest, starsRaw] = await Promise.all([
      fetchJson(root + "/manifest.json"),
      fetchJson(root + "/stars.json").catch(() => null),
    ]);
    const rawAirports = (starsRaw && starsRaw.airports) || {};
    const stars = expandBundleAirports(rawAirports);
    return { manifest, stars, meta: manifest };
  }

  // ---------------------------------------------------------------------------
  // Per-plan restriction edits (v3.0). A filled waypoint carries a frozen copy of
  // its published restriction in `pub`. Edits change only the plan's waypoint
  // fields; the loaded STAR database object is never touched.
  // ---------------------------------------------------------------------------
  const EDIT_FIELDS = ["typ", "alt", "min", "max", "spd"];
  function normField(k, v) {
    if (v == null) return "";
    const t = String(v).trim();
    if (k === "typ") return t || "at";
    if (t === "") return "";
    const n = Number(t.replace(/,/g, ""));
    return Number.isFinite(n) ? String(n) : t.toUpperCase();
  }
  function relevantFields(typ) {
    if (typ === "window") return ["typ", "min", "max", "spd"];
    if (typ === "none") return ["typ", "spd"];
    return ["typ", "alt", "spd"];
  }
  function publishedSnapshot(wp) {
    const o = {};
    for (const k of EDIT_FIELDS) o[k] = wp && wp[k] != null ? String(wp[k]) : "";
    if (!o.typ) o.typ = "at";
    return Object.freeze(o);
  }
  function withPublished(entry) {
    return (entry || []).map((w) => Object.assign({}, w, { pub: publishedSnapshot(w) }));
  }
  function editedFields(wp) {
    if (!wp || !wp.pub) return [];
    const cur = normField("typ", wp.typ);
    const pubTyp = normField("typ", wp.pub.typ);
    const keys = new Set(relevantFields(cur).concat(relevantFields(pubTyp)));
    const out = [];
    for (const k of EDIT_FIELDS) {
      if (!keys.has(k)) continue;
      if (normField(k, wp[k]) !== normField(k, wp.pub[k])) out.push(k);
    }
    return out;
  }
  function isEdited(wp) {
    return editedFields(wp).length > 0;
  }
  function resetToPublished(wp) {
    if (!wp || !wp.pub) return wp;
    const o = Object.assign({}, wp);
    for (const k of EDIT_FIELDS) o[k] = wp.pub[k];
    return o;
  }
  function describeRestriction(r) {
    if (!r) return "";
    const typ = r.typ || "at";
    const n = (v) => (v === "" || v == null ? "?" : Number(v).toLocaleString("en-US"));
    let a;
    if (typ === "none") a = "no altitude";
    else if (typ === "window") a = n(r.min) + "–" + n(r.max);
    else a = ({ at: "at ", at_or_above: "at or above ", at_or_below: "at or below " }[typ] || "") + n(r.alt);
    return r.spd !== "" && r.spd != null ? a + " / " + r.spd + " kt" : a;
  }

  global.CRJCifp = {
    publishedSnapshot,
    withPublished,
    editedFields,
    isEdited,
    resetToPublished,
    describeRestriction,
    gcNm,
    formatNm,
    starTitle,
    rwyTitle,
    expandStar,
    assembleArrival,
    flyingToEntry,
    cycleIsExpired,
    formatCycleDate,
    loadBundle,
    loadCanadaManifest,
    loadCanadaBundle,
    listAirports,
    listStars,
    listTransitions,
    listRunways,
    commonStartOptions,
  };
})(typeof globalThis !== "undefined" ? globalThis : window);
