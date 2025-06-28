from fastkml import kml
from shapely.geometry import Point
import json


def json_to_kml(json_file, kml_file):
    with open(json_file, 'r') as f:
        geotags = json.load(f)

    ns = '{http://www.opengis.net/kml/2.2}'
    k_doc = kml.KML()
    doc = kml.Document(ns=ns, id='docid', name='Drone Geotags', description='Generated Geotags')
    k_doc.append(doc)

    folder = kml.Folder(ns=ns, id='fid', name='Geotags', description='All Geotags')
    doc.append(folder)

    for tag in geotags:
        p = Point(tag['center_longitude'], tag['center_latitude'])
        placemark = kml.Placemark(
            ns=ns,
            id=str(tag['sector_id']),
            name=f"Sector {tag['sector_id']}",
            description=tag['timestamp']
        )
        placemark._geometry = p
        folder.append(placemark)

    with open(kml_file, 'w') as f:
        f.write(k_doc.to_string(prettyprint=True))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Convert Drone Geotag JSON to KML")
    parser.add_argument("json_file", help="Path to input geotag JSON file")
    parser.add_argument("kml_file", help="Path to output KML file")
    args = parser.parse_args()

    json_to_kml(args.json_file, args.kml_file)
