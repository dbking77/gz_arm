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


def i_rect(x, y):
    return (x*x + y*y)/12.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    args = parser.parse_args()

    # https://en.wikibooks.org/wiki/Statics/Moment_of_Inertia_(contents)
    # use m/12*(h^2 + w^2) for all links
    # 2,710kg/m3
    aluminum_density = 2710.0
    m = args.x*args.y*args.z * aluminum_density
    xx = i_rect(args.z, args.y)*m
    yy = i_rect(args.x, args.z)*m
    zz = i_rect(args.x, args.y)*m

    print(f"<mass>{m:0.5f}</mass>")
    print(f"<inertia>")
    print(f"  <ixx>{xx:0.5f}</ixx>")
    print(f"  <ixy>0</ixy>")
    print(f"  <ixz>0</ixz>")
    print(f"  <iyy>{yy:0.5f}</iyy>")
    print(f"  <iyz>0</iyz>")
    print(f"  <izz>{zz:0.5f}</izz>")
    print(f"</inertia>")


if __name__ == "__main__":
    main()
