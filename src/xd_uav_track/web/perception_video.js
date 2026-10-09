/* Browser/Node helpers. Video dimensions and interaction coordinates stay native. */
"use strict";
const PerceptionVideo = (() => {
  const MAX_BYTES = 16777216, MAX_PIXELS = 16777216;
  function bytes(data) {
    if (typeof data === "string") {
      const padding = data.endsWith("==") ? 2 : data.endsWith("=") ? 1 : 0;
      const decodedLength = Math.floor(data.length / 4) * 3 + Math.floor(data.length % 4 * 3 / 4) - padding;
      if (data.length > Math.ceil(MAX_BYTES / 3) * 4 || decodedLength > MAX_BYTES)
        throw new Error("PNG exceeds client byte limit");
      if (typeof Buffer !== "undefined") return new Uint8Array(Buffer.from(data, "base64"));
      const raw = atob(data), output = new Uint8Array(raw.length);
      for (let i = 0; i < raw.length; i++) output[i] = raw.charCodeAt(i);
      return output;
    }
    if (Array.isArray(data) || data instanceof Uint8Array) {
      if (data.length > MAX_BYTES) throw new Error("PNG exceeds client byte limit");
      if (Array.isArray(data) && data.some(value => !Number.isInteger(value) || value < 0 || value > 255))
        throw new Error("PNG contains invalid bytes");
      return new Uint8Array(data);
    }
    throw new Error("video image.data must be base64 or a byte array");
  }
  function pngMetadata(data) {
    const payload = bytes(data), signature = [137, 80, 78, 71, 13, 10, 26, 10];
    if (payload.length < 33 || signature.some((value, index) => payload[index] !== value) ||
        payload[12] !== 73 || payload[13] !== 72 || payload[14] !== 68 || payload[15] !== 82)
      throw new Error("video payload is not a PNG with an IHDR header");
    const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
    if (view.getUint32(8) !== 13) throw new Error("invalid PNG IHDR size");
    const width = view.getUint32(16), height = view.getUint32(20);
    if (!width || !height) throw new Error("PNG dimensions must be positive");
    if (width > MAX_PIXELS / height) throw new Error("PNG exceeds client pixel limit");
    return {bytes:payload, width, height};
  }
  function stampEqual(a, b) { return !!a && !!b && a.secs === b.secs && a.nsecs === b.nsecs; }
  function decode(bundle) {
    const targets = bundle && bundle.targets, image = bundle && bundle.image;
    if (!targets || !image || !image.header || !targets.header || !bundle.header)
      throw new Error("video bundle is missing frame metadata");
    if (!String(image.format).toLowerCase().includes("png")) throw new Error("video must use lossless PNG");
    const png = pngMetadata(image.data);
    if (png.width !== targets.image_width || png.height !== targets.image_height)
      throw new Error("PNG dimensions do not match native target dimensions");
    if (!stampEqual(image.header.stamp, targets.header.stamp) || !stampEqual(bundle.header.stamp, targets.header.stamp) ||
        image.header.seq !== targets.frame_seq ||
        image.header.frame_id !== targets.header.frame_id || bundle.header.frame_id !== targets.header.frame_id)
      throw new Error("video image and target metadata describe different frames");
    return Object.assign({}, bundle, {pngBytes:png.bytes, width:png.width, height:png.height});
  }
  // CSS object-fit:contain geometry, including horizontal/vertical black bars.
  function point(rect, width, height, clientX, clientY, clamp = false) {
    const scale = Math.min(rect.width / width, rect.height / height);
    if (!(scale > 0)) return null;
    const left = rect.left + (rect.width - width * scale) / 2, top = rect.top + (rect.height - height * scale) / 2;
    const x = (clientX - left) / scale, y = (clientY - top) / scale;
    if (!clamp && (x < 0 || y < 0 || x > width || y > height)) return null;
    return [Math.max(0, Math.min(width, x)), Math.max(0, Math.min(height, y))];
  }
  function region(start, end, width, height) {
    return [Math.min(start[0], end[0]) / width, Math.min(start[1], end[1]) / height,
      Math.abs(end[0] - start[0]) / width, Math.abs(end[1] - start[1]) / height];
  }
  function identity(frame) {
    const target = frame.targets;
    return {session_id:target.session_id, state_version:target.state_version, model_epoch:target.model_epoch,
      mode:target.mode, image_source:target.image_source, frame_seq:target.frame_seq, capture_stamp:{...target.header.stamp}};
  }
  return {MAX_BYTES, MAX_PIXELS, bytes, pngMetadata, decode, point, region, identity, stampEqual};
})();
if (typeof module !== "undefined") module.exports = PerceptionVideo;
else globalThis.PerceptionVideo = PerceptionVideo;
