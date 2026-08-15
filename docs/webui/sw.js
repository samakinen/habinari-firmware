/*
 * Offline cache for the commissioning page.
 *
 * A plant room is exactly the place with no signal, so the tool has to survive
 * being loaded once at the office and used a week later underground. The policy
 * is stale-while-revalidate: serve from the cache immediately, then refresh in
 * the background so the next visit is current.
 *
 * Bump CACHE_VERSION whenever index.html changes. Nothing else enforces it —
 * an installer running a stale page against a newer wire version would be told
 * so by the version check in index.html, but that is a diagnosis, not a fix.
 */

const CACHE_VERSION = "habinari-webui-v1";

const PRECACHE = [
  "./",
  "./index.html",
  "./manifest.webmanifest",
  "./icon.svg",
];

self.addEventListener("install", (event) => {
  event.waitUntil(
    caches.open(CACHE_VERSION)
      .then((cache) => cache.addAll(PRECACHE))
      .then(() => self.skipWaiting()));
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys()
      .then((names) => Promise.all(
        names.filter((name) => name !== CACHE_VERSION).map((name) => caches.delete(name))))
      .then(() => self.clients.claim()));
});

self.addEventListener("fetch", (event) => {
  const request = event.request;

  /* Only this origin, and only plain reads: anything else is not ours to
   * answer. */
  if (request.method !== "GET" || new URL(request.url).origin !== self.location.origin) {
    return;
  }

  event.respondWith((async () => {
    const cache = await caches.open(CACHE_VERSION);
    const cached = await cache.match(request, { ignoreSearch: true });

    const network = fetch(request)
      .then((response) => {
        if (response.ok) cache.put(request, response.clone());
        return response;
      })
      .catch(() => null);

    /* Cached copy wins on speed; the network copy lands in the cache for next
     * time. With no cache and no network there is nothing to serve, and the
     * browser's own offline page is a clearer message than anything here. */
    return cached || (await network) || Response.error();
  })());
});
