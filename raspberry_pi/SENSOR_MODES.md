# SENSOR MODES

The Raspberry Pi cameras we use advertise a few sensor modes to choose from:

## V2 Camera

```
[{'bit_depth': 10,
  'crop_limits': (1000, 752, 1280, 960),
  'exposure_limits': (37, None),
  'format': SRGGB10_CSI2P,
  'fps': 206.65,
  'size': (640, 480),
  'unpacked': 'SRGGB10'},

 {'bit_depth': 10,
  'crop_limits': (0, 0, 3280, 2464),
  'exposure_limits': (75, 5883419, None),
  'format': SRGGB10_CSI2P,
  'fps': 41.85,
  'size': (1640, 1232),
  'unpacked': 'SRGGB10'},

 {'bit_depth': 10,
  'crop_limits': (680, 692, 1920, 1080),
  'exposure_limits': (75, 11766829, None),
  'format': SRGGB10_CSI2P,
  'fps': 47.57,
  'size': (1920, 1080),
  'unpacked': 'SRGGB10'},

 {'bit_depth': 10,
  'crop_limits': (0, 0, 3280, 2464),
  'exposure_limits': (75, 11766829, None),
  'format': SRGGB10_CSI2P,
  'fps': 21.19,
  'size': (3280, 2464),
  'unpacked': 'SRGGB10'},

 {'bit_depth': 8,
  'crop_limits': (1000, 752, 1280, 960),
  'exposure_limits': (37, 5883414, None),
  'format': SRGGB8,
  'fps': 206.65,
  'size': (640, 480),
  'unpacked': 'SRGGB8'},

 {'bit_depth': 8,
  'crop_limits': (0, 0, 3280, 2464),
  'exposure_limits': (37, 5883414, None),
  'format': SRGGB8,
  'fps': 83.7,
  'size': (1640, 1232),
  'unpacked': 'SRGGB8'},

 {'bit_depth': 8,
  'crop_limits': (680, 692, 1920, 1080),
  'exposure_limits': (75, 5883419, None),
  'format': SRGGB8,
  'fps': 47.57,
  'size': (1920, 1080),
  'unpacked': 'SRGGB8'},

 {'bit_depth': 8,
  'crop_limits': (0, 0, 3280, 2464),
  'exposure_limits': (75, 11766829, None),
  'format': SRGGB8,
  'fps': 21.19,
  'size': (3280, 2464),
  'unpacked': 'SRGGB8'}]
```

There are four uncropped modes here: one each of 8 bit or 10 bit, and one each of full or 2x2 binned.

We'll take the 10 bit 2x2 binned one, and we add a bit to the width so that the image stride is a multiple of 16.

## V3 Wide Camera

```
[{'bit_depth': 10,
  'crop_limits': (768, 432, 3072, 1728),
  'exposure_limits': (9, None),
  'format': SRGGB10_CSI2P,
  'fps': 120.13,
  'size': (1536, 864),
  'unpacked': 'SRGGB10'},

 {'bit_depth': 10,
  'crop_limits': (0, 0, 4608, 2592),
  'exposure_limits': (13, 77208384, None),
  'format': SRGGB10_CSI2P,
  'fps': 56.03,
  'size': (2304, 1296),
  'unpacked': 'SRGGB10'},

 {'bit_depth': 10,
  'crop_limits': (0, 0, 4608, 2592),
  'exposure_limits': (26, 112015443, None),
  'format': SRGGB10_CSI2P,
  'fps': 14.35,
  'size': (4608, 2592),
  'unpacked': 'SRGGB10'}]
```

The first mode is a very fast center crop: more frame rate than we can use, and narrower than we want.

The third one is *all* the pixels, which is a lot.

The middle one is a 2x2 bin of the full sensor, which is what we want.

## Global Shutter Camera

The GS camera has only one mode:

```
  [{'bit_depth': 10,
  'crop_limits': (0, 0, 1456, 1088),
  'exposure_limits': (29, 15534385, None),
  'format': SRGGB10_CSI2P,
  'fps': 60.38,
  'size': (1456, 1088),
  'unpacked': 'SRGGB10'}]
```