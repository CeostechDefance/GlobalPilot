"""Küçük alanlar için düz yer yaklaşımı: enlem/boylam ile metre arası dönüşümler.

Planlayıcı ve görev betikleri aynı formülleri buradan alır (önceden fire_attack, water_leg, pist ve birakma'da ayrı ayrı
yazılıydı). Doğu ölçeği ilk noktanın enleminden alınır. Yerel x/y çerçevesi isteyenler rota.YerelCerceve'yi kullanır.
"""
import math

__all__ = ['M_PER_DEG', 'offset', 'along', 'distance_m', 'bearing_deg', 'turn_deg', 'track_errors']

M_PER_DEG = 111320.0


def offset(lat, lon, north, east):
    """Noktadan kuzeye north, doğuya east metre kaydırılmış nokta."""
    return lat + north / M_PER_DEG, lon + east / (M_PER_DEG * math.cos(math.radians(lat)))


def along(lat, lon, bearing_deg, distance, right=0.0):
    """Noktadan bearing_deg yönünde distance metre ileri, right metre sağa."""
    b = math.radians(bearing_deg)
    return offset(lat, lon, distance * math.cos(b) - right * math.sin(b), distance * math.sin(b) + right * math.cos(b))


def distance_m(lat1, lon1, lat2, lon2):
    return math.hypot((lat2 - lat1) * M_PER_DEG, (lon2 - lon1) * M_PER_DEG * math.cos(math.radians(lat1)))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Birinci noktadan ikinciye yön: kuzeyden saat yönünde, 0-360 derece."""
    return math.degrees(math.atan2((lon2 - lon1) * math.cos(math.radians(lat1)), lat2 - lat1)) % 360


def turn_deg(a, b):
    """a yönünden b yönüne en kısa dönüş açısı (derece, 0-180)."""
    return abs((b - a + 180.0) % 360.0 - 180.0)


def track_errors(lat, lon, ref, bearing):
    """Noktanın, ref'ten bearing yönünde geçen hatta göre boyuna (ileri +) ve yanal (sağa +) uzaklığı, m."""
    n = (lat - ref[0]) * M_PER_DEG
    e = (lon - ref[1]) * M_PER_DEG * math.cos(math.radians(ref[0]))
    b = math.radians(bearing)
    return n * math.cos(b) + e * math.sin(b), -n * math.sin(b) + e * math.cos(b)
