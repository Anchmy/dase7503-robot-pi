"""QR decoding helpers.

OpenCV's QRCodeDetector is tried first because it handled perspective and
blur better than zbar in our synthetic tests. pyzbar (ZBar) is kept as a
fallback since it often wins on clean, high-contrast captures.
"""

import cv2


def _candidates(gray):
    """Yield increasingly aggressive preprocessed variants of a gray image."""
    yield gray

    height, width = gray.shape[:2]
    if max(height, width) < 640:
        scale = 2
        yield cv2.resize(gray, (width * scale, height * scale),
                         interpolation=cv2.INTER_CUBIC)

    # Adaptive threshold copes with uneven lighting on the game field.
    yield cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 31, 5)


def decode_qr(image_bgr, max_width=640):
    """Return the first decoded QR string in the image, or None.

    Frames wider than max_width are downscaled first: on the Pi 5 this cut
    the per-frame cost from ~80 ms to ~25 ms, and a QR that fills a useful
    part of the frame still decodes at half resolution.
    """
    if image_bgr is None or image_bgr.size == 0:
        return None

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    width = gray.shape[1]
    if max_width and width > max_width:
        scale = max_width / width
        gray = cv2.resize(gray, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_AREA)
    detector = cv2.QRCodeDetector()

    for candidate in _candidates(gray):
        # OpenCV 4.6 throws a cv2.error (-215 convexHull assertion) on some
        # low-texture frames.  A single bad frame must not take the node down,
        # so every decode attempt is isolated.
        try:
            data, _points, _straight = detector.detectAndDecode(candidate)
        except cv2.error:
            continue
        if data:
            return data

    try:
        from PIL import Image
        from pyzbar.pyzbar import decode as zbar_decode
    except ImportError:
        return None

    for candidate in _candidates(gray):
        try:
            results = zbar_decode(Image.fromarray(candidate))
        except Exception:
            continue
        if results:
            return results[0].data.decode('utf-8', errors='replace')

    return None
