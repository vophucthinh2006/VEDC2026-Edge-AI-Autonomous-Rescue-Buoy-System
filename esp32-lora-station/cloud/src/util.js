const enc = new TextEncoder();

export const json = (obj, status = 200, headers = {}) =>
  new Response(JSON.stringify(obj), { status, headers: { "content-type": "application/json", ...headers } });

export const num = v => (typeof v === "number" && Number.isFinite(v) ? v : null);

// So sánh chuỗi bí mật không rò rỉ độ dài/thời gian: băm SHA-256 rồi so hai giá trị băm
export async function secretEquals(a, b) {
  const [ha, hb] = await Promise.all([
    crypto.subtle.digest("SHA-256", enc.encode(a)),
    crypto.subtle.digest("SHA-256", enc.encode(b)),
  ]);
  return crypto.subtle.timingSafeEqual(ha, hb);
}
