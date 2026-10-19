#!/usr/bin/env python3
"""Aviation Dynamics: Brazil's airports for destination search and approaches.

Builds data/world/airports_br.json from OurAirports (public domain:
airports.csv, runways.csv, airport-frequencies.csv) and the curated ILS
table below. Kept: Brazilian airports (large, medium, small) with at least
one open paved runway of 3,000 ft or more.

Each runway keeps its ends' coordinates when OurAirports has them; otherwise
the core places it at the airport reference point on its true heading (from
OurAirports, else from the runway number and the regional magnetic
variation estimate). ILSs: OurAirports' frequency records with a runway, plus
CURATED_ILS (each with its source). Every other runway gets an RNAV (GPS)
approach in the simulation.

    python3 tools/navdata/brazil.py [ourairports-csv-dir]

Without a directory the CSVs are fetched from the OurAirports data
repository on GitHub. Not for real-world navigation.
"""
import csv
import io
import json
import os
import re
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(REPO, 'data', 'world', 'airports_br.json')
RAW = 'https://raw.githubusercontent.com/davidmegginson/ourairports-data/main/'

# ILSs not in OurAirports, cross-checked where possible. Runway numbers are
# the current ones.
CURATED_ILS = {
    'SBKP': [('15', 'IKP', 110.30, 'Flight Plan Database listing (single secondary source; unverified)')],
    'SBGL': [('10', 'ITB', 109.30, 'Flight Plan Database listing (single secondary source; unverified)'),
             ('15', 'IGL', 110.30, 'Flight Plan Database listing; SBGL approach chart (SlideShare copy) agrees'),
             ('28', 'ILM', 111.50, 'Flight Plan Database listing; SBGL approach chart (SlideShare copy) agrees')],
    'SBBR': [('11L', 'IBR', 110.30, 'Flight Plan Database listing (single secondary source; may predate changes)'),
             ('11R', 'IDF', 109.90, 'Flight Plan Database listing (single secondary source; may predate changes)'),
             ('29L', 'IBS', 111.50, 'Flight Plan Database listing (single secondary source; may predate changes)'),
             ('29R', 'IND', 109.30, 'Flight Plan Database listing (single secondary source; may predate changes)')],
}
# The names people use for airports that OurAirports doesn't carry.
EXTRA_ALIASES = {
    'SBCF': 'Confins, BH', 'SBBH': 'Pampulha, BH', 'SBGR': 'Guarulhos', 'SBSP': 'Congonhas', 'SBKP': 'Viracopos',
    'SBGL': 'Galeão, Tom Jobim', 'SBRJ': 'Santos Dumont', 'SBBR': 'JK, Juscelino Kubitschek', 'SBFI': 'Iguaçu, Cataratas',
    'SBCT': 'Afonso Pena', 'SBPA': 'Salgado Filho', 'SBFZ': 'Pinto Martins', 'SBBE': 'Val de Cans', 'SBEG': 'Eduardo Gomes',
    'SBMT': 'Campo de Marte', 'SBJD': 'Jundiaí', 'SBSJ': 'São José dos Campos', 'SBCY': 'Marechal Rondon, Cuiabá',
    'SBMO': 'Zumbi dos Palmares', 'SBSV': 'Dois de Julho', 'SBRF': 'Guararapes', 'SBFL': 'Hercílio Luz',
}
PAVED = ('ASP', 'CON', 'PEM', 'BIT', 'PAV', 'TAR')
KIND = {'large_airport': 'L', 'medium_airport': 'M', 'small_airport': 'S'}


def read(name, folder):
    if folder:
        with open(os.path.join(folder, name), encoding='utf-8') as f:
            return list(csv.DictReader(f))
    with urllib.request.urlopen(RAW + name, timeout=120) as r:
        return list(csv.DictReader(io.StringIO(r.read().decode('utf-8'))))


def num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def reciprocal_fix(le, he):
    """The far end's name: as recorded when it is the reciprocal of the near
    end (numbers 18 apart, L/R swapped), else derived from the near end (a
    few records pair ends like 16/30)."""
    m1, m2 = re.match(r'(\d{1,2})([LRC]?)$', le), re.match(r'(\d{1,2})([LRC]?)$', he)
    if not m1:
        return he
    n1 = int(m1.group(1))
    if m2 and abs((int(m2.group(1)) - n1) % 36 - 18) <= 1:
        return he
    n2 = (n1 + 18 - 1) % 36 + 1
    side = {'L': 'R', 'R': 'L', 'C': 'C', '': ''}[m1.group(2)]
    return f'{n2:02d}{side}' if len(m1.group(1)) == 2 else f'{n2}{side}'


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else None
    airports = [a for a in read('airports.csv', folder) if a['iso_country'] == 'BR' and a['type'] in KIND]
    runways, freqs = {}, {}
    for r in read('runways.csv', folder):
        runways.setdefault(r['airport_ident'], []).append(r)
    freq_name = 'airport-frequencies.csv'
    for f in read(freq_name, folder if folder and os.path.exists(os.path.join(folder, freq_name)) else None):
        freqs.setdefault(f['airport_ident'], []).append(f)
    out = []
    for a in airports:
        rws = []
        for r in runways.get(a['ident'], []):
            length = num(r['length_ft'])
            if r['closed'] == '1' or not length or length < 3000 or not any(p in r['surface'].upper() for p in PAVED):
                continue
            if not r['le_ident'] or not r['he_ident']:
                continue
            he_ident = reciprocal_fix(r['le_ident'], r['he_ident'])
            le = [num(r['le_latitude_deg']), num(r['le_longitude_deg'])]
            he = [num(r['he_latitude_deg']), num(r['he_longitude_deg'])]
            le = [round(x, 6) for x in le] if None not in le else le
            he = [round(x, 6) for x in he] if None not in he else he
            rws.append({
                'ends': [r['le_ident'], he_ident],
                'le': le if None not in le else None,
                'he': he if None not in he else None,
                'hdg': num(r['le_heading_degT']),
                'len': round(length),
                'wid': round(num(r['width_ft']) or 98),
                'disp': [round(num(r['le_displaced_threshold_ft']) or 0), round(num(r['he_displaced_threshold_ft']) or 0)],
            })
        if not rws:
            continue
        ends = {e for r in rws for e in r['ends']}
        ils = []
        for f in freqs.get(a['ident'], []):
            m = re.match(r'RWY\s*(\w+)\s*-\s*(\w+)', f['description'] or '')
            if f['type'] == 'ILS' and m and m.group(1) in ends:
                ils.append({'rwy': m.group(1), 'id': m.group(2), 'mhz': num(f['frequency_mhz']),
                            'src': 'OurAirports airport-frequencies.csv (public domain)'})
        for rwy, ident, mhz, src in CURATED_ILS.get(a['ident'], []):
            if rwy in ends and not any(i['rwy'] == rwy for i in ils):
                ils.append({'rwy': rwy, 'id': ident, 'mhz': mhz, 'src': src})
        out.append({
            'icao': a['ident'], 'iata': a['iata_code'], 'name': a['name'], 'city': a['municipality'],
            'state': a['iso_region'].replace('BR-', ''), 'kind': KIND[a['type']], 'sched': a['scheduled_service'] == 'yes',
            'elev': round(num(a['elevation_ft']) or 0), 'lat': round(num(a['latitude_deg']), 6), 'lon': round(num(a['longitude_deg']), 6),
            'alias': ', '.join([k.strip() for k in a['keywords'].split(',') if k.strip() and '://' not in k]
                               + ([EXTRA_ALIASES[a['ident']]] if a['ident'] in EXTRA_ALIASES else [])),
            'rw': rws, 'ils': ils,
        })
    out.sort(key=lambda x: ('LMS'.index(x['kind']), x['icao']))
    doc = {
        'source': 'OurAirports (public domain, https://ourairports.com/data/); ILSs as cited per record',
        'note': 'Approximate; not for real-world navigation. Runways without end coordinates are placed at the '
                'airport reference point by the simulation.',
        'airports': out,
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, separators=(',', ':'))
        f.write('\n')
    n_ils = sum(len(a['ils']) for a in out)
    print(f'wrote {os.path.relpath(OUT, REPO)}: {len(out)} airports, {sum(len(a["rw"]) for a in out)} runways, {n_ils} ILSs')


if __name__ == '__main__':
    main()
