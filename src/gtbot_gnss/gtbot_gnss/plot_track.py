#!/usr/bin/env python3
"""
CSV 로그 → 지도 경로 HTML (+KML)
  python3 plot_track.py                 # ~/gnss_logs 안에서 가장 최근 CSV 사용
  python3 plot_track.py logs/xxx.csv    # 특정 파일
  python3 plot_track.py --kml           # Google Earth용 KML도 같이 생성
출력: 같은 이름의 .html (브라우저로 열기). 지도 타일은 인터넷 필요.
"""
import argparse, csv, glob, json, math, os, statistics, sys, webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
COLOR = {"RTK-FIXED": "#1a9850", "RTK-FLOAT": "#f46d43", "NONE": "#4575b4"}
LABEL = {"RTK-FIXED": "RTK FIXED (cm급)", "RTK-FLOAT": "RTK FLOAT (수십cm)", "NONE": "보정 없음 (m급)"}

def haversine(a, b):
    R = 6371000.0
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dp, dl = math.radians(b[0] - a[0]), math.radians(b[1] - a[1])
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))

def load(path):
    pts = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                lat, lon = float(r["lat"]), float(r["lon"])
            except ValueError:
                continue
            if r["fix"] in ("NoFix", "Time") or (abs(lat) < 1e-6 and abs(lon) < 1e-6):
                continue
            pts.append(dict(t=r["local_time"], rtk=r["rtk"] or "NONE", sats=int(r["sats"]),
                            lat=lat, lon=lon, alt=float(r["alt_m"]), hacc=float(r["hAcc_m"])))
    return pts

def stats(pts):
    dist = sum(haversine((pts[i-1]["lat"], pts[i-1]["lon"]), (pts[i]["lat"], pts[i]["lon"])) for i in range(1, len(pts)))
    n = len(pts); cnt = {k: sum(p["rtk"] == k for p in pts) for k in COLOR}
    return dict(points=n, start=pts[0]["t"], end=pts[-1]["t"], dist_m=dist,
                pct={k: 100.0 * v / n for k, v in cnt.items()},
                hacc_med=statistics.median(p["hacc"] for p in pts), hacc_min=min(p["hacc"] for p in pts))

def write_html(pts, st, out, title):
    data = json.dumps(pts, ensure_ascii=False)
    legend = "".join(f'<div><span style="background:{c}"></span>{LABEL[k]} {st["pct"][k]:.0f}%</div>' for k, c in COLOR.items())
    html = f"""<!doctype html><html><head><meta charset="utf-8"><title>{title}</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
html,body,#map{{height:100%;margin:0}}
.panel{{position:absolute;top:10px;right:10px;z-index:1000;background:#fff;padding:10px 12px;border-radius:6px;
box-shadow:0 1px 6px rgba(0,0,0,.3);font:13px/1.5 system-ui,sans-serif;max-width:260px}}
.panel span{{display:inline-block;width:14px;height:14px;border-radius:3px;vertical-align:middle;margin-right:6px}}
.panel b{{display:block;margin-bottom:4px}}
</style></head><body><div id="map"></div>
<div class="panel"><b>{title}</b>
{st['start']} → {st['end']}<br>점 {st['points']}개, 거리 {st['dist_m']:.0f} m<br>
hAcc 중앙값 {st['hacc_med']:.2f} m, 최소 {st['hacc_min']:.3f} m<hr style="margin:6px 0">{legend}</div>
<script>
const pts={data};
const COLOR={json.dumps(COLOR)};
const map=L.map('map');
L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',{{maxZoom:19,attribution:'&copy; OpenStreetMap'}}).addTo(map);
const bounds=[];
for(let i=1;i<pts.length;i++){{
  const a=pts[i-1],b=pts[i];
  L.polyline([[a.lat,a.lon],[b.lat,b.lon]],{{color:COLOR[b.rtk]||'#888',weight:4,opacity:.9}}).addTo(map);
  bounds.push([b.lat,b.lon]);
}}
pts.forEach(p=>{{
  L.circleMarker([p.lat,p.lon],{{radius:3,color:COLOR[p.rtk]||'#888',fillOpacity:.8,weight:1}})
   .bindPopup(`${{p.t}}<br>${{p.rtk}} / 위성 ${{p.sats}}<br>hAcc ${{p.hacc.toFixed(3)}} m<br>${{p.lat.toFixed(7)}}, ${{p.lon.toFixed(7)}}<br>고도 ${{p.alt.toFixed(1)}} m`).addTo(map);
}});
const s=pts[0],e=pts[pts.length-1];
L.marker([s.lat,s.lon]).bindPopup('시작 '+s.t).addTo(map);
L.marker([e.lat,e.lon]).bindPopup('끝 '+e.t).addTo(map);
map.fitBounds(bounds.length?bounds:[[s.lat,s.lon]],{{padding:[30,30]}});
if(bounds.length<2) map.setView([s.lat,s.lon],18);
</script></body></html>"""
    open(out, "w").write(html)

def write_kml(pts, out, title):
    def seg_style(k, c):
        return f'<Style id="{k}"><LineStyle><color>ff{c[5:7]}{c[3:5]}{c[1:3]}</color><width>4</width></LineStyle></Style>'
    styles = "".join(seg_style(k, c) for k, c in COLOR.items())
    pms, cur, coords = [], None, []
    def flush():
        if cur and len(coords) > 1:
            pms.append(f'<Placemark><name>{LABEL[cur]}</name><styleUrl>#{cur}</styleUrl><LineString><tessellate>1</tessellate>'
                       f'<coordinates>{" ".join(coords)}</coordinates></LineString></Placemark>')
    for p in pts:
        if p["rtk"] != cur:
            flush(); cur = p["rtk"]; coords = coords[-1:] if coords else []
        coords.append(f'{p["lon"]},{p["lat"]},{p["alt"]}')
    flush()
    open(out, "w").write(f'<?xml version="1.0" encoding="UTF-8"?><kml xmlns="http://www.opengis.net/kml/2.2"><Document>'
                         f'<name>{title}</name>{styles}{"".join(pms)}</Document></kml>')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?"); ap.add_argument("--kml", action="store_true")
    ap.add_argument("--open", action="store_true", help="생성 후 브라우저로 열기")
    a = ap.parse_args()
    path = a.csv or (sorted(glob.glob(os.path.join(os.path.expanduser("~/gnss_logs"), "*.csv")), key=os.path.getmtime) or [None])[-1]
    if not path: sys.exit("CSV 파일 없음")
    pts = load(path)
    if not pts: sys.exit(f"{path}: fix 된 위치가 없음")
    st = stats(pts); title = os.path.splitext(os.path.basename(path))[0]
    out = os.path.splitext(path)[0] + ".html"
    write_html(pts, st, out, title)
    print(f"[html] {out}")
    if a.kml:
        k = os.path.splitext(path)[0] + ".kml"; write_kml(pts, k, title); print(f"[kml]  {k}")
    print(f"점 {st['points']}개  {st['start']} ~ {st['end']}  거리 {st['dist_m']:.0f} m")
    print("  " + "  ".join(f"{LABEL[k]} {st['pct'][k]:.0f}%" for k in COLOR))
    print(f"  hAcc 중앙값 {st['hacc_med']:.3f} m, 최소 {st['hacc_min']:.3f} m")
    if a.open: webbrowser.open("file://" + os.path.abspath(out))

if __name__ == "__main__":
    main()
