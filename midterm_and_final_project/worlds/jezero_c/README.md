# Jezero_C Terrain Data

This directory contains HiRISE-derived terrain assets for the Jezero Crater landing
site region — specifically, a 40×40 m patch centered 300 m east and 20 m south of
Octavia E. Butler Landing (Perseverance rover's Sol 1 position).

## What's committed to git

| Path | Size | Purpose |
|---|---|---|
| `dtm/jezero_c_patch_40m_raw.tif` | 7 KB | 40×40 Float32, cropped from full tile |
| `dtm/jezero_c_patch_40m_normalized.tif` | 5 KB | Same patch, elevation 0-based (subtract min) |
| `dtm/jezero_c_heightmap_65.tif` | 9 KB | Padded to 65×65 (2^6+1) for Gazebo heightmap compatibility |
| `dtm/jezero_c_heightmap_65.png` | 1 KB | 8-bit PNG fallback of the same |
| `ortho/jezero_c_patch_40m.tif` | 26 KB | Grayscale HiRISE RED filter ortho, 40×40 m patch |
| `ortho/jezero_c_texture_257.png` | 30 KB | Upsampled to 257×257, Mars-ochre tinted for use as Gazebo texture |
| `terrain_mesh/jezero_terrain.obj` | 718 KB | Triangulated mesh, 4225 vertices, 8192 triangles, UV+normals |
| `terrain_mesh/jezero_terrain.mtl` | 280 B | Material definition referencing the texture |

## What's NOT committed (gitignored due to size)

| Path | Size | How to re-obtain |
|---|---|---|
| `dtm/DTM_MOLAtopography_DeltaGeoid_Jezero_C_Edited_affine_1m_Eqc_latTs0_lon0.tif` | 120 MB | See "Download commands" below |
| `ortho/ESP_045994_1985_Edited_affine_25cm_Eqc_latTs0_lon0.tif` | 1.6 GB | See "Download commands" below |
| `dtm/*.xml` | 17 KB ea. | Companion metadata, re-downloadable |

## Data provenance

- **Source:** USGS Astrogeology Science Center, Mars 2020 Terrain Relative Navigation
  (TRN) mosaic products, Jezero Crater landing site.
- **Format:** Affine-aligned to CTX reference frame, Equirectangular Mars 2000
  Sphere IAU (radius 3,396,190 m).
- **Release date:** November 10, 2022.
- **Significance:** The same DTM product was used in Perseverance's onboard flight
  software during EDL (Entry, Descent, and Landing) for hazard map generation.

## Download commands (re-obtain raw TIFFs)

```bash
cd worlds/jezero_c/

# DTM (120 MB)
wget -c -O dtm/DTM_MOLAtopography_DeltaGeoid_Jezero_C_Edited_affine_1m_Eqc_latTs0_lon0.tif \
  https://asc-pds-services.s3.us-west-2.amazonaws.com/mosaic/mars2020_trn/HiRISE/DTM_MOLAtopography_DeltaGeoid_Jezero_C_Edited_affine_1m_Eqc_latTs0_lon0.tif

# Nadir ortho (1.6 GB)
wget -c -O ortho/ESP_045994_1985_Edited_affine_25cm_Eqc_latTs0_lon0.tif \
  https://asc-pds-services.s3.us-west-2.amazonaws.com/mosaic/mars2020_trn/HiRISE/ESP_045994_1985_Edited_affine_25cm_Eqc_latTs0_lon0.tif
```

Use the `-c` flag to resume if the download is interrupted.

## Full S3 bucket index

https://asc-pds-services.s3.us-west-2.amazonaws.com/mosaic/mars2020_trn/HiRISE/

Contains 7 stereo pairs covering Jezero (Jezero_C is the landing site tile):
- Jezero_C (center, our tile) — Octavia E. Butler Landing is inside this tile
- Jezero_N (north / delta approach)
- Jezero_DL (delta lobe — high slope, not usable for DiffDrive)
- Jezero_E, Jezero_W
- Jezero_CR_N, Jezero_CR_S (crater rim)

Each has a nadir observation (`ESP_XXXXXX_XXXX_...tif`) and an off-nadir
(`..._offNadir.tif`). Use nadir for clean orthoimagery; off-nadir has foreshortening
distortion when used as texture.

## Pipeline to regenerate derivatives from raw TIFFs

If you clone fresh and need to regenerate the small files:

1. Download raw TIFFs per above
2. Crop 40×40 m patch:
   ```bash
   gdal_translate -projwin 4591177.544 1093318.023 4591217.544 1093278.023 \
     dtm/DTM_MOLAtopography_...tif dtm/jezero_c_patch_40m_raw.tif
   gdal_translate -projwin 4591177.544 1093318.023 4591217.544 1093278.023 \
     ortho/ESP_045994_1985_...tif ortho/jezero_c_patch_40m.tif
   ```
3. Normalize elevations + pad heightmap + generate ochre texture:
   see `scripts/` in the project root for Python scripts
4. Convert to mesh:
   ```bash
   python3 scripts/dtm_to_obj.py
   ```

## Crop patch bounding box

Chosen deliberately NOT to overlap the Butler Landing scar (avoiding descent-stage
crash debris and parachute artifacts).

```
X (easting):  4,591,178 to 4,591,218 m   (40 m wide)
Y (northing): 1,093,278 to 1,093,318 m   (40 m tall)
```

Butler Landing projection coordinates (Mars 2000 Sphere, R = 3,396,190 m):
- lat 18.4446° → y = R * rad(lat) = 1,093,298.1 m
- lon 77.4509° → x = R * rad(lon) = 4,590,878.7 m

Our patch is 300 m east + 20 m south of the landing.

## Measured terrain characteristics

- Full Jezero_C tile: 7.06 × 14.44 km
- Elevation range (full tile): −2665 to −2396 m (below Mars 2000 Sphere reference)
- Our 40×40 m patch: elevation range 3.08 m (after normalization: 0.0 to 3.08 m)
- Mean slope: ~4.4% (~2.5°)
- StdDev: 0.888 m
- Well within DiffDrive's 10-15° planar limit
