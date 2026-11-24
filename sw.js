/* CRJ descent planner — network-first so a new deploy loads on the next online open. */
const CACHE = "crj-descent-v3.0-classic";
const PRECACHE = [
  "./",
  "./index.html",
  "./planner.js",
  "./manifest.webmanifest",
  "./apple-touch-icon.png",
  "./icon-180.png",
  "./icon-192.png",
  "./icon-192-maskable.png",
  "./icon-512.png",
  "./icon-512-maskable.png",
  "./favicon.ico",
  "./favicon-32.png",
  "./version.json",
  "./cifp.js",
  "./cifp/manifest.json",
  "./cifp/stars.json.gz",
  "./canada/manifest.json",
  "./canada/stars.json",
  "./airport_elevations.json",
];

self.addEventListener("install", (event) => {
  event.waitUntil(
    caches
      .open(CACHE)
      .then((cache) => cache.addAll(PRECACHE))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    (async () => {
      const keys = await caches.keys();
      await Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)));
      await self.clients.claim();
    })()
  );
});

async function networkFirst(request) {
  const cache = await caches.open(CACHE);
  try {
    const fresh = await fetch(request, { cache: "no-store" });
    if (fresh && fresh.ok) {
      cache.put(request, fresh.clone());
    }
    return fresh;
  } catch (err) {
    const cached = (await cache.match(request)) || (await cache.match("./index.html")) || (await cache.match("./"));
    if (cached) return cached;
    throw err;
  }
}

self.addEventListener("fetch", (event) => {
  const req = event.request;
  if (req.method !== "GET") return;
  const url = new URL(req.url);
  if (url.origin !== self.location.origin) return;
  event.respondWith(networkFirst(req));
});
