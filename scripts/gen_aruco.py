#!/usr/bin/env python3

# MIT License

# Copyright (c) 2024 Derek King

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser("generate arcuo tags")
    parser.add_argument("id", type=int, help="tag ID to generate")
    parser.add_argument("--size", type=int,
                        help="tag size in pixels", default=60)
    parser.add_argument("--out", "-o", help="output filename")
    parser.add_argument("--show", "-S", help="show image")
    args = parser.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    border = 1
    sz = args.size
    img = np.zeros((sz, sz, 1), dtype="uint8")
    cv2.aruco.drawMarker(dictionary, args.id, sz, img, 1)

    if args.out:
        cv2.imwrite(args.out, img)
    if args.show:
        cv2.imshow("ArUCo Tag", img)
        cv2.waitKey(0)


if __name__ == "__main__":
    main()
