import cv2
import numpy as np
from pathlib import Path

RAW = Path('/home/jatin-satyam/ses598-apollo17/data/rgb/mag138')

img35 = cv2.imread(str(RAW / 'AS17-138-21035HR.png'))
img36 = cv2.imread(str(RAW / 'AS17-138-21036HR.png'))
img37 = cv2.imread(str(RAW / 'AS17-138-21037HR.png'))

for img, name in [(img35,'21035'), (img36,'21036'), (img37,'21037')]:
    print(f'{name}: shape={img.shape}, dtype={img.dtype}')

orb = cv2.ORB_create(nfeatures=4000)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

def match_pair(a, b, name_a, name_b):
    kpa, da = orb.detectAndCompute(cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), None)
    kpb, db = orb.detectAndCompute(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY), None)
    if da is None or db is None or len(da)==0 or len(db)==0:
        print(f'{name_a}<->{name_b}: no descriptors')
        return 0
    matches = bf.match(da, db)
    good = [m for m in matches if m.distance < 60]
    print(f'{name_a}<->{name_b}: total={len(matches)} good(dist<60)={len(good)}')
    if len(good) >= 8:
        pts_a = np.float32([kpa[m.queryIdx].pt for m in good])
        pts_b = np.float32([kpb[m.trainIdx].pt for m in good])
        _, mask = cv2.findHomography(pts_a, pts_b, cv2.RANSAC, 5.0)
        inliers = int(mask.sum()) if mask is not None else 0
        print(f'  homography inliers: {inliers}')
        return inliers
    return 0

inliers_35_36 = match_pair(img35, img36, '21035', '21036')
inliers_36_37 = match_pair(img36, img37, '21036', '21037')
inliers_35_37 = match_pair(img35, img37, '21035', '21037')

print()
if inliers_35_36 + inliers_36_37 < 10:
    print('CONCLUSION: 21036 has negligible overlap with neighbors - geometrically isolated, skip registration')
else:
    print(f'CONCLUSION: 21036 has {inliers_35_36+inliers_36_37} combined inliers - may be recoverable')
