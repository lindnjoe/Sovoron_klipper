// Self-unregistering service worker.
//
// This page is an embedded iframe served fresh by nginx; a caching service
// worker provided no benefit and kept serving a STALE page (e.g. old CSS),
// which is hard to clear from the browser. Tear the worker down and reload any
// controlled clients so they fetch current assets straight from nginx.
self.addEventListener('install', function () {
    self.skipWaiting();
});

self.addEventListener('activate', function (event) {
    event.waitUntil((async function () {
        try {
            // Drop any caches a previous version of this worker may have created.
            if (self.caches && caches.keys) {
                var keys = await caches.keys();
                await Promise.all(keys.map(function (k) { return caches.delete(k); }));
            }
        } catch (e) { /* ignore */ }
        await self.registration.unregister();
        var clients = await self.clients.matchAll({ type: 'window' });
        clients.forEach(function (c) {
            if (c.navigate) { c.navigate(c.url); }
        });
    })());
});

// While still active, never serve from cache — always go to the network.
self.addEventListener('fetch', function (event) {
    event.respondWith(fetch(event.request, { cache: 'no-store' }));
});
