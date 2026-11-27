#!/usr/bin/python3
# http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-15-lightmaps/
# https://notblackmagic.com/bitsnpieces/mr-buggy3/
# requires python >= 3.9
import itertools
import time, datetime, sys, os
import pygame
from pygame.constants import *
from OpenGL.GLU import *
from OpenGL.GL import *

from model import *
from team_driver import TeamDriver

import pixy2sim, numpy as np

"""©2018 Tudor Popovici"""
"""Simple Autonomous Driving Simulator platform with 3D Graphics acceleration using OpenGL"""
"""module useful for visualizing scenarios and testing autonomous driving algorithms"""

log = logging.Logger('sim.log')

# performance stats depending on detail (on Linux Mint 21.2 Cinnamon, Intel© Core™ i5-2520M CPU @ 2.50GHz × 2, NVIDIA Corporation GF119M [NVS 4200M])
# no multisample/antialias, just essential (top left image): ~200fps => x3.3 faster than realtime
# no multisample/antialias, but full display (all quarters): ~100fps => x1.6 faster than realtime

class NxpTrack:
    r"""
    Draws using OpenGL an NXP Track based on a configuration string with following syntax
    The charaters are track segment drawing commands or intesection segments end markers:
    - short straight line
    = long straight line
    > start line
    ~ wave segment (represents 3 real small wave segments connected correctly)
    / left 45 degree turn
    \ right 45 degree turn
    * intersection node (it is followed by the right turn branch segments)
    , intersection branch separator (terminates the right and forward branches)
    . intersection terminator (terminates the left turn branch
    (right,forward,left) intersection paths; comma separates the 3 directions

    Extra restrictions: make sure that the intersection syntax is respected since no checks are made!
        *right turn segment,forward turn segment,left turn segment.
    The * is moving model coordinates trasformation matrix to intersection center,
            rotates 90 degrees to the right, saves on stack the transformation matrix,
            and translates away to the edge of the intersection to continue with the right branch
    The , is restoring from the stack the model coordinates tranformation matrix,
            back to the intersetion center after rotation (where it was pushed on stack)
            and then coninues with a 90 degree left rotation and saving on stack the new coordinate matrix,
            followed by translating away to the edge of the intersection
            to continue with the next intersection branch drawing
    The . is just popping out the last intersection center coordinates
                (to continue a previous intersection or finish a track)
    """
    TRACK_LIB = {
        "demoS":  r'>\\\\=\\\\',
        "demoM":  r'>-*-\\=\\\\,-=////=//-,.',
        "demoM0": r'>=\\\\==\\\\',
        "demoXL": r'>=//=//\\=\\=////==~~==////\\=*-=////=//-,,=\\\\////=.',
        "demoxl": r'<=\\\\////=*=//\\\\==~~==\\\\=//=//\\=\\=,-=////,-\\=.', #reverse demoXL
        "demoAC": r'>/-*-//////,-,-//\\/~~/////\\/=.',
        "nxp21f": r'>=\\=-*-====\\=\\==,-===//~//=\\//==//=//\\-,.',
        "nxp22f": r'>=//~~\\=////===//=\\=-*-=//=//=//=-,,-=//=.',
        "nxp23r": r'>=\\//\\=\\=\\//=-*-=\\,--===//~~//==//\\//=,.',
        "nxp23fr":r"<\\=//=//=\\=\\=~~===-\\--=//==*-=*==//==\\=~~=\\//=\\,-=//=//=//=-,.,-=//=//=//=-,.",
        "nxp23f": r">//=\\//=~~=//==\\==*-=//=//=//=-,,-=*-=//=//=//=-,,==\\=--//-===~~=//=//=\\=\\=//..",
    }
    TRACK_DEF = "nxp23f"

    @staticmethod
    def generate_blocks():
        def draw_obj(vs, qls, cs, ls, t=None, r=None):
            """
            Draws an object that uses a colection of vertices as support for a set of differently colored collections of polygons and/or lines
            Similar to the Wavefront Obj format, but more simple, without support for normals and textures, just plain colors
            """
            if vs:  # if vertices are provided, then drawing quadrilaterals with different colors is attempted
                glFrontFace(GL_CCW)  # quadrilaterals will be visible from the side on which
                                        # the order of the vertices is trigonometric (GPU specific ssiue)
                glEnableClientState(GL_VERTEX_ARRAY)  # prepare to handle vertices as indices from a common array
                                                        # (it makes easier to stich together neighbouring polygons)
                glVertexPointer(3, GL_FLOAT, 0, vs)  #np.array(vs, dtype=np.float32))
                for ql, c in zip(qls, cs):
                    if c:
                        glColor3f(*c)
                        glDrawElements(GL_QUADS, len(ql), GL_UNSIGNED_INT, ql)  #np.array(ql, dtype=np.uint32))
                if 1 and ls:
                    for l in ls:
                        glColor3f(*l[0])
                        glDrawElements(GL_LINES, len(l[1]), GL_UNSIGNED_INT, l[1])
                glDisableClientState(GL_VERTEX_ARRAY)
            if t: glTranslate(*t)  # move cursor on model map, if requested
            if r: glRotate(*r)  # rotate map at cursor, if requested

        draw_segment_limits = False # draw segment limits (for visual check)
        L = 0.720  # long straight segment length (normal straights, start marked straights and full waves (made of 3 small real wave segments)
        w = 0.020  # black marking line width
        W = 0.550  # full track width (including black marking side borders)
        h = 0.001  # relative height at which track is going to be drawn
        r = 0.170  # inner track curve radius (distance from curve center point and inner side of the track segment),
        # also the short straight segment length (used to compensate 90 degree turns when connecting to intersections
        R = 0.400  # inner track wave radius (full waves are geometrically made by concatenating \
        # two opposing arc bands that in reality have a length equal to a straight segment,
        # but which is split in thirds in the actual track kits
        # (the order of the thirds is relevant when building the track with the actual kit, for aesthetic purposes)
        # (in simulation wave are available only in full, not in thirds, for simplicity)
        xs = 0.540  # start marker offset from beginning of the straight long segment
        xh = 0.037  # offset from the center of the lane for the half start markers
        xH = 0.094  # width across the lane of each start marker half
        cS = 2  # carpet building square width (used for avoiding issues with shading very large polygons
        cs = 9  # number of carpet squares away from (0,0) in each cardinal direction (semi width)
        fn = 8*4  # number of segments for approximating a full circle (more means additional effort on the GPU)
        bc = (0.0, 0.0, 0.0)  # border marking color (dark/black)
        tc = (1.0, 1.0, 1.0)  # track lane color (bright/white)
        dc = (0.5, 0.0, 0.0)  # segment split color
        cc = tc #(0.9, 0.9, 0.9)  # carpet color
        rw = 0.0365 # wheel radius
        ww = 0.030  # wheel width
        cf = 0.261  # forward length from rear axle
        cb = 0.036  # backward length from rear axle
        cl = 0.104  # lateral half-length
        ch = 0.079  # height to top floor
        from math import sin, cos, sqrt, asin
        def rng(s, step, e):
            c =  int((e - s)/step)
            step = (e -s)/c
            return (s+step*i for i in range(c+1))
        def rng0(s,step,e):
            _=tuple(rng(s,step,e))
            return _[:-1] if _[-1]==e else _
        def arc0(cx, cy, z, r, As, Ae):
            return ((r*cos(a)+cx, r*sin(a)+cy, z) for a in rng0(As,math.pi/fn,Ae))

        def arc(cx, cy, z, r, As, Ae):
            return ((r*cos(a)+cx, r*sin(a)+cy, z)for a in  rng(As,math.pi/fn,Ae))

        def off(xyz, x, y, z = 0):
            return ((X+x,Y+y, Z+z) for X, Y, Z in xyz)

        def adj(xyz, x, y, z = 1):
            return ((X * x, Y * y, Z * z) for X, Y, Z in xyz)

        # calculating wave line primitive template
        dl=sqrt(R*R-L*L/16);#lateral offset for curve arc center
        dL=L/4;#longitudinal offset for curve arc center
        A=asin(dL/R);#curve arc semi-length in radians
        dA=-math.pi/2;#curve arc center orientation
        As=dA-A;#curve arc start on circle
        Ae=dA+A;#curve arc end on circle
        v0=tuple(arc0(dL,dl,h,R,As,Ae));#convex arc, first half
        v2=(*v0, *off(adj(v0,1,-1), 2*dL, 0),(L,0,h));#wave line

        wave = (*off(v2, 0, -W/2),*off(v2, 0, -W/2+w),*off(v2, 0, W/2-w),*off(v2, 0, W/2),)

        tcx = r + W / 2
        turnl = (
                *arc(0,tcx,h,r, -math.pi/2,-math.pi/4),
                *arc(0,tcx,h,r+w,-math.pi/2,-math.pi/4),
                *arc(0,tcx,h,r+W-w,-math.pi/2,-math.pi/4),
                *arc(0,tcx,h,r+W,-math.pi/2,-math.pi/4))
        ap = len(turnl)>>2
        turnr = tuple(adj((*turnl[3*ap:], *turnl[2*ap:3*ap], *turnl[ap:2*ap], *turnl[:ap]), 1, -1))

        from itertools import chain
        tq = (tuple((
                    *chain(*((i,i+ap,ap+(i+1)%ap,(i+1)%ap) for i in range(ap))),
                    *chain(*((i+2*ap,i+3*ap,3*ap+(i+1)%ap,2*ap+(i+1)%ap) for i in range(ap)))
                ,)),
                tuple(chain(*((i+ap,i+2*ap,2*ap+(i+1)%ap,ap+(i+1)%ap) for i in range(ap)))),
        )

        ap = len(wave) >> 2

        tw = (tuple((
            *chain(*((i, i + ap, ap + (i + 1) % ap, (i + 1) % ap)[::-1] for i in range(ap))),
            *chain(*((i + 2 * ap, i + 3 * ap, 3 * ap + (i + 1) % ap, 2 * ap + (i + 1) % ap)[::-1] for i in range(ap)))
            ,)),
              tuple(chain(*((i + ap, i + 2 * ap, 2 * ap + (i + 1) % ap, ap + (i + 1) % ap)[::-1] for i in range(ap)))),
        )
        blocks = {
            ' ': lambda : draw_obj(None, None, None, None,
                           (W/2, 0, 0)),
            ',': lambda : draw_obj(((W/2,-W/2,h),(W/2,W/2,h),), (), (),
                           ((dc, (0,1)),) if draw_segment_limits else None,
                           (W/2, 0, 0)),
            '<': lambda : draw_obj(((0, -W / 2, h), (0, -W / 2 + w, h), (0, W / 2 - w, h), (0, W / 2, h),
                            (L, -W / 2, h), (L, -W / 2 + w, h), (L, W / 2 - w, h), (L, W / 2, h),
                            (xs, -xh-xH, h), (xs, -xh, h), (xs, xh, h), (xs, xh+xH, h),
                            (xs+w, -xh-xH, h), (xs+w, -xh, h), (xs+w, xh, h), (xs+w, xh+xH, h),
                            (xs, -W/2+w, h), (xs, W/2-w, h), (xs+w, -W/2+w, h), (xs+w, W/2-w, h),
                            (xs, -W/2 + w, h), (xs, -xh-xH, h), (xs + w, -W/2+w, h), (xs + w, -xh - xH, h),
                            (xs, W/2 - w, h), (xs, xh+xH, h), (xs + w, W/2-w, h), (xs + w, xh + xH, h),
                            (xs, -xh, h), (xs, xh, h), (xs + w, -xh, h), (xs + w, xh, h),),
                           ((0, 4, 5, 1,  2, 6, 7, 3,  8, 12, 13, 9,  10, 14, 15, 11),
                            (1, 16, 17, 2,  18, 5, 6, 19,  21, 20, 22, 23,  24, 25, 27, 26,  29, 28, 30, 31)),
                           (bc, tc), ((dc, (4,7)),) if draw_segment_limits else None,
                           (L, 0, 0)),
            '>': lambda : draw_obj(((0, -W / 2, h), (0, -W / 2 + w, h), (0, W / 2 - w, h), (0, W / 2, h),
                            (L, -W / 2, h), (L, -W / 2 + w, h), (L, W / 2 - w, h), (L, W / 2, h),
                            (L-xs-w, -xh - xH, h), (L-xs-w, -xh, h), (L-xs-w, xh, h), (L-xs-w, xh + xH, h),
                            (L-xs-w + w, -xh - xH, h), (L-xs-w + w, -xh, h), (L-xs-w + w, xh, h), (L-xs-w + w, xh + xH, h),
                            (L-xs-w, -W / 2 + w, h), (L-xs-w, W / 2 - w, h), (L-xs-w + w, -W / 2 + w, h), (L-xs-w + w, W / 2 - w, h),
                            (L-xs-w, -W / 2 + w, h), (L-xs-w, -xh - xH, h), (L-xs-w + w, -W / 2 + w, h), (L-xs-w + w, -xh - xH, h),
                            (L-xs-w, W / 2 - w, h), (L-xs-w, xh + xH, h), (L-xs-w + w, W / 2 - w, h), (L-xs-w + w, xh + xH, h),
                            (L-xs-w, -xh, h), (L-xs-w, xh, h), (L-xs-w + w, -xh, h), (L-xs-w + w, xh, h),),
                           ((0, 4, 5, 1, 2, 6, 7, 3, 8, 12, 13, 9, 10, 14, 15, 11),
                            (1, 16, 17, 2, 18, 5, 6, 19, 21, 20, 22, 23, 24, 25, 27, 26, 29, 28, 30, 31)),
                           (bc, tc), ((dc, (4, 7)),) if draw_segment_limits else None,
                           (L, 0, 0)),
            '=': lambda : draw_obj(((0, -W / 2, h), (0, -W / 2 + w, h), (0, W / 2 - w, h), (0, W / 2, h),
                            (L, -W / 2, h), (L, -W / 2 + w, h), (L, W / 2 - w, h), (L, W / 2, h)),
                           ((0, 4, 5, 1,  2, 6, 7, 3), (1, 5, 6, 2)),
                           (bc, tc), ((dc, (4,7)),) if draw_segment_limits else None,
                           (L, 0, 0)),
            '-': lambda : draw_obj(((0, -W / 2, h), (0, -W / 2 + w, h), (0, W / 2 - w, h), (0, W / 2, h),
                            (r, -W / 2, h), (r, -W / 2 + w, h), (r, W / 2 - w, h), (r, W / 2, h)),
                           ((0, 4, 5, 1,  2, 6, 7, 3), (1, 5, 6, 2)),
                           (bc, tc), ((dc, (4,7)),) if draw_segment_limits else None,
                           (r, 0, 0)),
            '~': lambda : draw_obj(wave,tw,(bc, tc), ((dc, ((len(wave)>>2)-1,3*(len(wave)>>2)-1)),) if draw_segment_limits else None,
                           (L, 0, 0)),
            '*': lambda : draw_obj(((0, -W / 2, h), (0, -W / 2 + w, h), (0, W / 2 - w, h), (0, W / 2, h),
                            (w, -W / 2, h), (w, -W / 2 + w, h), (w, W / 2 - w, h), (w, W / 2, h),
                            (W-w, -W / 2, h), (W-w, -W / 2 + w, h), (W-w, W / 2 - w, h), (W-w, W / 2, h),
                            (W, -W / 2, h), (W, -W / 2 + w, h), (W, W / 2 - w, h), (W, W / 2, h)),
                           ((0, 4, 5, 1,  2, 6, 7, 3,  8, 12, 13, 9,  10, 14, 15, 11), (1, 5, 6, 2,  9, 13, 14, 10,  4, 8, 11, 7)),
                           (bc, tc),
                           ((dc, (0,12)),) if draw_segment_limits else None,
                           (W/2, 0, 0), (-90, 0,0,1)),
            '/': lambda : draw_obj(turnl, tq,(bc, tc),
                           ((dc, ((len(turnl)>>2)-1,3*(len(turnl)>>2)-1)),) if draw_segment_limits else None,
                           (tcx*sin(math.pi/4), tcx*(1-sin(math.pi/4)),0), (45, 0,0,1)),
            '\\': lambda : draw_obj(turnr, tq,
                           (bc, tc),
                            ((dc, ((len(turnr)>>2)-1,3*(len(turnr)>>2)-1)),) if draw_segment_limits else None,
                            (tcx * sin(math.pi / 4), -tcx * (1 - sin(math.pi / 4)), 0), (-45, 0, 0, 1)),
            'C': lambda : draw_obj((-cs, -cs, 0.0, cs, -cs, 0.0, cs, cs, 0.0, -cs, cs, 0.0), ((0, 1, 2, 3),), (tc,),  None),
            'c': lambda : draw_obj(tuple(chain(*((x*cS, y*cS, 0) for y in range(-cs, cs+1) for x in range(-cs, cs+1)))),
                           (tuple(chain(*((i+j*(2*cs+1), i+1+j*(2*cs+1), i+1+(2*cs+1)*(j+1), i+(2*cs+1)*(j+1)) for i in range(2*cs) for j in range(2*cs)))),),
                           (cc,),
                           None, #(((255,0,0),(0,(2*cs+1)**2-1),),) if ds else None
                           ), # seems that very large poligons have shading issues... so carpet is better made from smaller ones...
            'm': lambda : draw_obj(((cf, cl, 0), (cf, -cl, 0), (cf, cl, ch), (cf, -cl, ch),
                           (-cb, cl, 0), (-cb, -cl, 0), (-cb, cl, ch), (-cb, -cl, ch),),
                           ((3, 2, 6, 7),), ((0.0, 0.0, 1.0),),None)
        }
        OPENGL_DISPLAYLIST_BUGFIX = False # Set this to True if you encounter bad track layouts (each segment gets doubled and the track breaks... encountered on a nVidia system under Windows 11...)
        if not OPENGL_DISPLAYLIST_BUGFIX: # when the fix is not needed lets generate calls to display lists that we prepare here for each objecte
            # but this is tricky because now we need to send as argument to the new lambda the result of the original lambda: seems that it can be done in more ways...
            def make_gl_display_list(v): list_descriptor = glGenLists(1); glNewList(list_descriptor, GL_COMPILE); v(); glEndList(); return list_descriptor  # make a display list based on an object and get the list number of later calling
            blocks = {k: (lambda x: lambda : glCallList(x))(make_gl_display_list(v)) for k, v in blocks.items()} # incremental improvement over the attempt on next line            
            # blocks = {k: (lambda x: lambda : glCallList(x))(make_gl_display_list(v)) for k, v in blocks.items()} # build over second answer in https://stackoverflow.com/questions/10452770/python-lambdas-binding-to-local-values
            # blocks = {k: y(make_gl_display_list(v)) for y in [lambda x: lambda : glCallList(x)] for k, v in blocks.items()} # second answer in https://stackoverflow.com/questions/10452770/python-lambdas-binding-to-local-values
            # blocks = {k: lambda x=make_gl_display_list(v): glCallList(x) for k, v in blocks.items()} # lambda with default parameter (first answer, best appreciated, but easier to break by sending a value to the default param on call)           
        deltas = {
            '-': (r, 0),
            '=': (L, 0),
            '~': (L, 0),
            '>': (L, 0),
            '*': (W / 2, 0),
            ',': (W / 2, 0),
            '/': ((r + W / 2) * math.pi / 4, (r + W / 2)),
            '\\': ((r + W / 2) * math.pi / 4, -(r + W / 2)),
        }
        return blocks, deltas


    blocks = None # track building blocks (dictionary of function to draw track segments)
    deltas = None # track running vectors (dictionary of arc lengths and arc radius for forward displacement, 0 radius means straight displacement)
    def __init__(self, trk):
        """
        NXP Track generator class (graphical representation for simulation built based on a config string)
        :param trk: the track configuration string stating all the segments used to build it in a specific order
        """
        self.lst = None
        self.trk = trk
        print('Track config:', trk)
    def draw(self) -> None:
        """
        Called to draw using opengl the track in the model space

        Since preparing the building blocks of the track requires an available opengl context, the static fields for
        blocks are initialized on the first call to draw.
        Since the automatic driving path for demo purposes uses geometry calculations done by opengl that can also
        be generated only after building the track blocks and the track
        :return: Nothing to return, just draws the track...
        """
        NxpTrack.blocks, NxpTrack.deltas = (NxpTrack.blocks, NxpTrack.deltas) if NxpTrack.blocks else NxpTrack.generate_blocks()
        self.lst = self.lst if self.lst else self.build_track_display_list(self.trk)
        glCallList(self.lst)

    def trace(self,
              s:((float, float, float), str, (float, float, float)),
              p:(float, float, float),
              N:{(float, float):(((float, float, float), str, (float, float, float)),)}):
        """
        Method that attempts to trace the correct run on the track
        :param s: current track segment descriptor
        :param p: previous node in the trace (used to avoid stepping back and to go forward in intersections)
        :param N: map of junction points (model space coordinates) and inbound segment descriptors
        :return: nothing, but it updates the path string and the start position mark
        """
        if not p:
            # initializing trace...
            self.st = s[0][:2]
            self.path = ""
        else:
            # logging in path the movement vector
            if s[2] == 1:
                if s[1] =='/':
                    self.path += '\\'
                elif s[1]=='\\':
                    self.path += '/'
                else:
                    self.path += s[1]
            else:
                self.path += s[1]

        if p and s[0][:2] == self.st: return # recursion ends when reaching the starting point

        def fwd(y):
            """
            Helper function to decide how to step forward on track (difficult in intersections)
            :param y:
            :return:
            """
            x = N[y[0][:2]]
            #print('?', x, y, p)
            if len(x) == 4:
                for d in x:
                    if d[0][:2] != p[:2] and (d[0][2] == p[2] or min(d[0][2], p[2])+180==max(d[0][2], p[2])):
                        return d
                else:
                    print("?????");exit()
            elif y[1] in ('*', ','):
                for d in x:
                    if d[0][:2] != p[:2] and (d[0][2] == y[0][2] or min(d[0][2], y[0][2])+180==max(d[0][2], y[0][2])):
                        return d
                else:
                    print("?????");
                    exit()
            else:
                if len(x)<2:
                    print('Track is not closed/looping???!')
                    return None
                return x[0] if x[0][0][:2] != y[0][:2] and (p is None or x[0][0][:2] != p[:2]) else x[1]
        n = fwd(s)
        if n:
            self.trace(n, s[0], N) # trace forward!!!
    def build_track_display_list(self, trk):
        b = self.build_track(trk)
        #print('box', b, b[0]+b[1], b[2]+b[3])
        lst = glGenLists(1); glNewList(lst, GL_COMPILE_AND_EXECUTE)  # execute is essential for being able to use glGetFloatv
        b = self.build_track(trk,(-(b[0]+b[1])/2,-(b[2]+b[3])/2,0), 0 if b[1]-b[0] > b[3] -b[2] else 90)
        #print('box', b, b[0] + b[1], b[2] + b[3])
        self.W = b[1] - b[0]
        self.H = b[3] - b[2]
        glEndList()
        nr = 2
        self.nodes = tuple((((round(s[0],nr), round(s[1],nr), s[2]), c, (round(d[0],nr), round(d[1],nr), d[2])) for s,c,d in self.nodes))
        #self.nodes = tuple(((s, c, d) for s,c,d in self.nodes))
        self.nodes = tuple((((tuple((0 if 0 ==x else x for x in s)), c, tuple((0 if 0 ==x else x for x in d)))) for s,c,d in self.nodes))

        N = {}
        for s, c, d in self.nodes:
            if s[:2] not in N: N[s[:2]] = []
            if d[:2] not in N: N[d[:2]] = []
            N[s[:2]].append((d, c, 0))
            N[d[:2]].append((s, c, 1))
        #print('\n'.join(map(str, self.nodes)));print('\n'.join(map(str, N.items()))) # debug segment junction locations
        if 1:
            self.trace(((round(self.start[0],nr), round(self.start[1],nr), self.start[2]), '>', 0), None, N);
            print('Track trace: ', self.path)
        #exit()
        return lst

    def build_track(self, trk:str, t:(float, float, float)=None, r:(float, float, float)=None) -> (float, float, float, float):
        """
        Build the track and also record the junctions of each track segment for calculating the track
        running path
        :param trk: track configuration string listing segments and intersections
        :param t: translation to be applied before starting to draw the track (used for centering the track)
        :param r: rotation to be applied before drawing the track (used for setting the longer side horizontally)
        :return: the minimum and maximums for the coordinates x and y, respectively (x, X, y, Y)
        that are reached by the track center line (useful for centering and rotating the track in the model space)
        """
        glPushMatrix()  # store initial model view
        if 1: # draw carpet, not really needed since clearing the background with white (search glClear), but can be usefull for lightning variation experiments
            NxpTrack.blocks['c']()
        if r:
            glRotate(r, 0, 0, 1)  # rotate map in model space

        if t:
            glTranslate(*t)

        self.waypoints = []
        self.nodes = []
        def gc() -> (float, float, float):
            """
            Get rounded (up to millimiter) coordinates of the current drawing point in the model space
            :return:
            """
            co = glGetFloatv(GL_MODELVIEW_MATRIX)
            return (round(co[3, 0], 3), round(co[3, 1], 3), round(np.degrees(np.arctan2(co[0, 1], co[0, 0]))))

        self.start = gc() # init start point for the tracks that might not have a start segment
        for s in trk:
            co = gc()
            self.waypoints.append(co)
            if len(self.nodes) and self.nodes[-1][2] is None: self.nodes[-1][2] = co
            if False:
                pass
            elif s == '*': # fork (follows right turn
                self.nodes.append([co, s, None])
                NxpTrack.blocks[s]()
                co = gc()
                if self.nodes[-1][2] is None: self.nodes[-1][2] = co
                glPushMatrix()
                self.nodes.append([co, ',', None])
                NxpTrack.blocks[' ']()
            elif s == ',': # next for turn (first forward, next left)
                #if self.nodes[-1][2] is None: self.nodes[-1][2] = gc()
                glPopMatrix()
                self.nodes.append([gc(), s, None])
                glRotate(90, 0,0,1)
                glPushMatrix()
                NxpTrack.blocks[s]()
            elif s == '.':
                #if self.nodes[-1][2] is None: self.nodes[-1][2] = gc()
                glPopMatrix()
            elif s in NxpTrack.blocks:
                self.nodes.append([co, s, None])
                if s == '>':
                    self.start = co
                NxpTrack.blocks[s]()
        if len(self.nodes) and self.nodes[-1][2] is None: self.nodes[-1][2] = gc()
        glPopMatrix()  # reset model view

        #print('wayp', self.waypoints)
        x,y,_,X,Y,_=*self.waypoints[0], *self.waypoints[0]
        for xx, yy, _ in self.waypoints[1:]:
            if xx<x: x = xx;
            if yy<y: y = yy;
            if X<xx: X = xx;
            if Y<yy: Y = yy;

        return x, X, y, Y

class Sim3DEnvironment:
    """Adjust race environment conditions"""

    def __init__(self, parent, width=636, height=None, lightness=0.5, sunlight=True):
        """edit constructor implementation to adapt lighting conditions"""
        self.parent = parent
        self.track = NxpTrack(NxpTrack.TRACK_LIB[NxpTrack.TRACK_DEF])
        self.width = width
        self.height = height if height else int(width * 3 / 4)
        self.screen_width = 2 * self.width
        self.screen_height = 2 * self.height
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (0, 0)

        pygame.init()
        
        multisample = 0 # attempt anti aliasing (obvious results for values > 2 up to limits depending on GPU, ussualy 16; the higher the better qulity, but slower) 

        if multisample: # >=3 on my pc makes things look nicer, but for some reason in accelerated mode it fails intersections, so clearly there is an issue with the timing...
            pygame.display.gl_set_attribute(GL_MULTISAMPLEBUFFERS, 1)
            pygame.display.gl_set_attribute(GL_MULTISAMPLESAMPLES, multisample)
            
        # print(pygame.font.get_fonts()) #list all available fonts!
        self.font = pygame.font.SysFont('monospace', 16)
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height)
                                              , OPENGL | DOUBLEBUF | NOFRAME | OPENGLBLIT # | FULLSCREEN
                                              , vsync=0 # vsync=1 makes it impossible running in accelerated mode because of syncronization to actual display device
                                              )
        if multisample:
            glEnable(GL_MULTISAMPLE)
        
        ambient_intensity = 0.00
        difuse = (lightness, lightness, lightness, 1.0)
        ambient = (ambient_intensity, ambient_intensity, ambient_intensity, 1.0)
        d, h = 4, 2.5 # light spacing around (0,0,0) model space and height above track
        # comment out light sources to vary the lighting conditions!
        Sim3DEnvironment.place_light(GL_LIGHT0, (-d, -d, h), ambient, difuse, sunlight)
        Sim3DEnvironment.place_light(GL_LIGHT1, (d, d, h), ambient, difuse, sunlight)
        Sim3DEnvironment.place_light(GL_LIGHT2, (-d, d, h), ambient, difuse, sunlight)
        if 0:
            Sim3DEnvironment.place_light(GL_LIGHT3, (d, -d, h), ambient, difuse, sunlight)
        else:
            print("South-East room light is OFF!")

        glEnable(GL_LIGHTING)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)  # most obj files expect to be smooth-shaded
        glEnable(GL_CULL_FACE)

        self.viewport_read_buffer = (GLubyte * (3 * self.width * self.height))(0)

    @staticmethod
    def place_light(light, position, ambient, difuse, parallel):
        glLightfv(light, GL_POSITION, (*position, 0.0 if parallel else 1.0))
        glLightfv(light, GL_AMBIENT, ambient)
        glLightfv(light, GL_DIFFUSE, difuse)
        glEnable(light)

    def display_text(self, txt, x=0, y=0, align_right=True
                     , foreground_color_bgr=(255, 255, 255)
                     , background_color_bgr=(0, 0, 0),
                     alpha = 255):
        self.set_viewport(0)  # top-left
        text = self.font.render(txt, True, foreground_color_bgr, background_color_bgr)
        raster = pygame.surfarray.array3d(text)
        raster = np.flip(np.rot90(raster), 0)
        if 0: # displaying text with alpha did not work...
            raster = np.pad(raster, ((0,0),(0,0),(0,1)), mode="constant", constant_values=alpha)
            raster = raster.astype(dtype=np.uint8)
            self.draw_raster(raster, (x, y), align_right, mode=GL_BGRA)
        else:
            self.draw_raster(raster, (x, y), align_right, mode=GL_BGR)

    def place_obj(self, x, y, z, r, obj_id, local_translation=None, local_rotation=None):
        glTranslate(x, y, z)
        glRotate(r, 0, 0, 1)
        if local_translation is not None:
            glTranslate(local_translation[0], local_translation[1], local_translation[2])
        if local_rotation is not None:
            glRotate(local_rotation, 0, 0, 1)
        NxpTrack.blocks[obj_id]()
        if local_rotation is not None:
            glRotate(-local_rotation, 0, 0, 1)
        if local_translation is not None:
            glTranslate(-local_translation[0], -local_translation[1], -local_translation[2])
        glRotate(-r, 0, 0, 1)
        glTranslate(-x, -y, -z)

    @staticmethod
    def draw_calibration_line(d, l, w, h=0.0, col=(1.0, 1.0, 1.0)):
        glColor3f(*col)
        glBegin(GL_POLYGON)
        #glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(d + w, -l / 2, h)
        #glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(d + w, l / 2, h)
        #glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(d, l / 2, h)
        #glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(d, -l / 2, h)
        glEnd()

    @staticmethod
    def draw_calibration_pattern(c:CalibrationPattern, x:float, y:float, r:float=0.0) -> None:
        """
        Draw a calibration pattern at the location (x, y)with the specified rotation r relative to the current position
        :param c: calibration pattern instance containing drawing parameters (dimensions)
        :param x: x translation from the current drawing point
        :param y: y translation...
        :param r: r (degrees)
        :return:
        """
        glPushMatrix() # saving on stack the coordinates of current point
        glTranslate(x, y, c.z) # repositioning relative to current point
        glRotate(r, 0, 0, 1)
        if 0: # optionaly draw the supporting white background calibrating sheets
            Sim3DEnvironment.draw_calibration_line(0, 0.7, 1.0)
            Sim3DEnvironment.draw_calibration_line(1.0, 1.0, 0.7)
            glTranslate(0, 0, 0.001)
        # the near horizontal calibration line
        Sim3DEnvironment.draw_calibration_line(c.d, c.l, c.w, c.z, (0, 0, 0))
        # the far horizontal calibration line
        Sim3DEnvironment.draw_calibration_line(c.D, c.L, c.W, c.z, (0, 0, 0))
        glPopMatrix() #restoring current point

    def get_lower_left_corner_of_viewport(self, i:int) -> (int, int):
        """
        Get the lower left coordinates of the specified quarter of the screen
        :param i:  the quarter identifier (0-3) as in the following map:
                2 3
                0 1
        :return: coordinate pair (x, y) of the lower left corner of the specified screen quarter
        """
        c = divmod(i, 2)
        return self.width * c[1], self.height * c[0]

    def read_viewport(self, i:int, mode=GL_BGR):
        """
        Utility to read as a raster the viewport specified by param i in the format specified by mode
        :param i: index of the viewport to be read ( 0 bottom left, 3 top right)
        :param mode: usual options: GL_BGR GL_LUMMINANCE
        :return: the raster matrix (2d or 3d depending on mono or colour mode selected)
        """
        glReadPixels(*self.get_lower_left_corner_of_viewport(i)
                     , self.width, self.height
                     , mode, GL_UNSIGNED_BYTE, self.viewport_read_buffer)
        if GL_LUMINANCE == mode:
            return np.flip(np.array(self.viewport_read_buffer, dtype=np.uint8).reshape(self.height, self.width), 0).astype(dtype=np.uint8)
        else:
            return np.flip(np.array(self.viewport_read_buffer, dtype=np.uint8).reshape(self.height, self.width, 3), 0).astype(dtype=np.uint8)

    def imwrite(self, viewport_no=2, fn='snap.png'):
        """
        Save a viewport in an image file for later reference
        :param viewport_no: the view port number (0-3)
        :param fn: the local file name (or full path)
        :return: Nothing
        """
        cv2.imwrite(fn, self.read_viewport(viewport_no))

    @staticmethod
    def draw_raster(raster, xy=None, use_bottom_right_corner=False, mode=GL_BGR):
        """
        Draw a raster (image) with the lower left corner starting at a specified point
        :param raster: 2d/3d matrix containing pixel values (byte formatted)
        :param xy: lower left x,y coordinates
        :param use_bottom_right_corner: align to right....
        :param mode: GL_BGR, GL_RGB, GL_LUMMINANCE, ...
        :return:
        """
        raster = np.flip(raster, 0) # in opengl images are vertically flipped...
        if xy is not None:
            if use_bottom_right_corner:
                glWindowPos2s(xy[0] - raster.shape[1], xy[1])
            else:
                glWindowPos2s(xy[0], xy[1])
        else:
            glWindowPos2s(0, 0)

        if len(raster.shape) > 2:
            raster = np.pad(raster, ((0,0),(0,raster.shape[1]%4),(0,0)), mode='constant', constant_values=(255,))
            glDrawPixels(raster.shape[1], raster.shape[0]
                         , mode, GL_UNSIGNED_BYTE, raster)
        else:
            raster = np.pad(raster, ((0,0),(0,raster.shape[1]%4)), mode='constant', constant_values=(255,))
            glDrawPixels(raster.shape[1], raster.shape[0]
                         , GL_LUMINANCE, GL_UNSIGNED_BYTE, raster)

    def set_viewport(self, i:int):
        """
        Set the viewport of the quarter split screen, in which to render 3d graphics
        :param i: quarter index (0 lower left)
        :return:
        """
        glViewport(*self.get_lower_left_corner_of_viewport(i)
                   , self.width, self.height)

    def sim_calibrate(self, car):
        """Simulates the automatic calibration of the camera positioning for perspective to orto transform"""
        print('calibrating')
        cp = car.calibration_pattern
        ol = (car.x, car.y, car.a)    # remember initial car position
        cl = (0, 0, 0)    # calibration area coordinates and orientation
        car.x, car.y, car.a = cl  # move car in the calibration area
        self.set_viewport(2)  # top-left
        glClearColor(1.0, 1.0, 1.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.place_camera(car.x, car.y, 1 * math.degrees(car.a), car.camera)    # simulation observing using the car camera
        # self.draw_scene([])    # redrawing car (might be annoying, but if done properly occlusion is more realistic)
        Sim3DEnvironment.draw_calibration_pattern(cp, *cl)
        self.display_text('calibration - press space to continue', 0, 0, False)
        if 0: # for simple camera (not pixy2)
            cp.calibrate(self.read_viewport(2))
        else: # for pixy2
            raster = self.read_viewport(2)
            pixy2sim.process_frame(raster, int(0), car.camera.pixy_out)
            if 0:
                import matplotlib.pyplot as plt
                print(raster.shape)
                plt.imshow(raster, cmap='gray')
                plt.show()
                plt.imshow(car.camera.pixy_out, cmap='gray')
                plt.show()
            cp.calibrate(car.camera.pixy_out)
        pygame.display.flip()  # switch the double buffer to the newly refreshed one

        pygame.time.wait(1)

        car.x, car.y, car.a = ol # restore car position in enviroment
        print('perspective warp matrix', car.calibration_pattern.M)

    @staticmethod
    def place_camera(x_m:float, y_m:float, azimuth_deg:float, camera: Camera, local_translation = None, local_rotation = None):
        """
        Simple camera placement in the projection space to observe the model space
        :param x_m: coordinate in meters on x axis on track from where to observe
        :param y_m: coordinate in meters on y axis on track from where to observe
        :param azimuth_deg: azimuth orientation of the camera from x axis
        :param camera: camera details (height, inclination, field of view, aspect ration)
        :param local_translation: additional translation (before applying camera details)
        :param local_rotation: additional rotation (after local translation, before camera details)
        :return:
        """
        Sim3DEnvironment.place_camera_adv(x_m, y_m, azimuth_deg + camera.rotation_deg
                                          , camera.height_m, camera.fwd_displacement_m, camera.inclination_deg
                                          , None, camera.aspect_ratio, camera.hfov_deg, local_translation=local_translation, local_rotation=local_rotation)

    @staticmethod
    def place_camera_adv(x_m, y_m, azimuth_deg, z_m, d_m, inclination_deg, ortho_width_m=None, aspect_ration=636.0 / 416.0
                         , hfov_deg=62.2, near_m=0.1, far_m=20.0, local_translation = None, local_rotation = None):
        """
        Advanced camera placement (frustrum creation) in the projection space to observe the model space
        :param x_m: coordinate in meters on x axis on track from where to observe
        :param y_m: coordinate in meters on y axis on track from where to observe
        :param azimuth_deg: azimuth orientation of the camera from x axis
        :param z_m: coordinate in meters on z axis on track from where to observe
        :param d_m:
        :param inclination_deg:
        :param ortho_width_m:
        :param aspect_ration:
        :param hfov_deg:
        :param near_m:
        :param far_m:
        :param local_translation:
        :param local_rotation:
        :return:
        """
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        if ortho_width_m is None:
            gluPerspective(hfov_deg, aspect_ration, near_m, far_m)
        else:
            w = ortho_width_m
            h = w / aspect_ration
            glOrtho(-w / 2, w / 2, -h / 2, h / 2, near_m, far_m + z_m)

        glRotate(inclination_deg, 1, 0, 0)
        if local_rotation is not None:
            glRotate(-local_rotation, 0, 1, 0)
        if local_translation is None:
            glTranslate(0, 0, d_m)
        else:
            glTranslate(local_translation[0], local_translation[1], local_translation[2]+d_m)
        glRotate(-azimuth_deg, 0, 1, 0)
        glRotate(-90, 1, 0, 0)
        # glRotate(0, 0, 1, 0)
        glRotate(90, 0, 0, 1)
        glTranslate(-x_m, -y_m, -z_m)
        #glEnable(GL_DEPTH_TEST) # globally controlled

    def draw_scene(self, cars, draw_trail=False):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        for i in range(len(cars)):
            c = cars[i]
            self.place_obj(c.x, c.y, 0, math.degrees(c.a),'m', (0, c.lateral_translate_m, 0), c.local_rotate_deg)
            if draw_trail:
                c.markers = c.markers[-1000:]
                cm = c.markers[::5]
                glEnableClientState(GL_VERTEX_ARRAY)  # prepare to handle vertices as indices from a common array
                # (it makes easier to stich together neighbouring polygons)
                glVertexPointer(3, GL_FLOAT, 0, tuple(itertools.chain(*cm)))  # np.array(vs, dtype=np.float32))
                glColor3f(1.0,0.0,0.0)
                glDrawElements(GL_LINES, len(cm), GL_UNSIGNED_INT, tuple(range(len(cm))))
                glDisableClientState(GL_VERTEX_ARRAY)
        self.track.draw()


    def draw_split_screen(self, cars:(Car,), observe=True, msg = None):
        '''
        Draws the main screen split in quarters (here is most of the display configuration code)
        :param cars: list of objects derived from Car that need to be displayed (just the first object is processed in current implementation)
        :param observe: flag set by the simulator to distinguish between slowmotion refresh and actual frames that need to be processed
        :return: nothing, just draws the scene and triggers line detection and forwards the detected object to the car driver to be observed
        '''
        glClearColor(1.0, 1.0, 1.0, 1.0) # use white as background (doesn't need white carpet simulation)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        c = cars[0]
        self.set_viewport(2)  # top-left (outer car)
        self.place_camera(c.x, c.y, 1 * math.degrees(c.a)
                          , c.camera, local_translation = (c.lateral_translate_m, 0, 0), local_rotation=c.local_rotate_deg)
        if 1: # draw the car model to simulate occlusion in the camera captured image
            self.draw_scene(cars)
        else: # just draw the scene, without the car
            self.draw_scene([])

        raster_raw = self.read_viewport(2) # capture the car mounted camera image persepective

        if observe: # executed on every simulated camera snapshot (not on slow motion inter-frames)
            c.camera.take_snapshot(raster_raw, c.t) # use pixy2 firmware to process current perspective
            # send to the driver the car object and the camers scan results
            pixy_out_topview = c.calibration_pattern.perspective_to_ortho(c.camera.pixy_out)  # view from the top (orto) transformation

            if 1: # adnotate the original pixy image with the original reported vectors (not top-view transformed)
                pixy_out = np.stack([c.camera.pixy_out] * 3, axis=-1)  # monochrome to BGR 3 channel conversion
                for inte in c.camera.pixy_intes:  # annotating intersections with blue circles in the raster
                    cv2.circle(pixy_out, (inte[1], inte[2]), 6,  (255, 0, 0),2)
                for line in c.camera.pixy_lines:  # annotating lines with green segments in the raster
                    cv2.line(pixy_out, ( line[1], line[2]), (line[3], line[4]),(0, 255, 0), 4)  # draw the actual line segment in the raster
                if c.camera.pixy_vector:
                    pv = c.camera.pixy_vector
                    cv2.line(pixy_out,(pv[1], pv[2]),(pv[3], pv[4]),(0, 0, 255), 2)
                c.camera.pixy_out_annotated = pixy_out
            if 1: # adnotate the top-view image with the transformed reported vectors
                pixy_out_topview = np.stack([pixy_out_topview] * 3, axis=-1)  # monochrome to BGR 3 channel conversion
                for inte in c.camera.pixy_intes:  # annotating intersections with blue circles in the raster
                    p = c.calibration_pattern.cam_coord_to_orto_int(c.calibration_pattern.M, inte[1], inte[2])
                    inte.append(p) # number of incident segments and point coordinates
                    cv2.circle(pixy_out_topview, p, 6,  (255, 0, 0),2)
                for line in c.camera.pixy_lines:  # annotating lines with green segments in the raster
                    p1, p2 = (c.calibration_pattern.cam_coord_to_orto_int(c.calibration_pattern.M, line[1], line[2]),
                              c.calibration_pattern.cam_coord_to_orto_int(c.calibration_pattern.M, line[3], line[4]),)
                    line.append((p1, p2))  # prepping line list for calculating the raster locations where to add the line indices later
                    cv2.line(pixy_out_topview, p1, p2,(0, 255, 0), 4)  # draw the actual line segment in the raster
                if c.camera.pixy_vector:
                    pv = c.camera.pixy_vector
                    p1, p2 =(c.calibration_pattern.cam_coord_to_orto_int(c.calibration_pattern.M, pv[1], pv[2]),
                            c.calibration_pattern.cam_coord_to_orto_int(c.calibration_pattern.M, pv[3], pv[4]))
                    pv.append((p1, p2))
                    cv2.line(pixy_out_topview, p1, p2, (0, 0, 255), 2)

            c.camera.pixy_out_topview_annotated = pixy_out_topview
            c.markers.append([c.x,c.y, 0.1])
            c.driver.observe(c, [c.camera.pixy_vector, c.camera.pixy_lines, c.camera.pixy_intes])

        if 1:
            self.set_viewport(1)  # bottom-right (top ortho)
            self.place_camera_adv(0, 0, 90, 1.3, 0.0, 90,
                                  max((self.track.H+0.7)*self.width/self.height,self.track.W+0.7))
            self.draw_scene(cars, True)

        # self.set_viewport(0)  # bottom-left (needed only for 3d rendering, not for raster draw)
        if 1 and c.camera.pixy_out_annotated is not None:  # display in the top-right corner the internal pixy2 camera reduced resolution data used for line detection
            Sim3DEnvironment.draw_raster(c.camera.pixy_out_annotated, (self.width + ((self.width - c.camera.pixy_out_annotated.shape[1]) >> 1), self.height))

        if 1 and c.camera.pixy_out_topview_annotated is not None:
            if 1:# raster text adnotations need to be drawn first to remain on top
                for line in c.camera.pixy_lines:
                    # placing line indices on middle point of lines
                    self.display_text(str(line[0]), (line[5][0][0]+line[5][1][0]) >> 1,
                                      c.camera.pixy_out_topview_annotated.shape[0] - ((line[5][0][1] + line[5][1][1]) >> 1), False)
            Sim3DEnvironment.draw_raster(c.camera.pixy_out_topview_annotated) # draw the orto view raster (top view, trapezoidal looking snapshot)

        pygame.display.flip()  # switch the display double buffer to the newly refreshed one


class Sim(DynamicPhysicalModel):
    keyboard_buffer = []

    def __init__(self, monitor_fps_min=25, slow_motion_scale=0.0, width=636, height=None, lightness=0.75, max_speed_m_per_s=1.0):
        super(Sim, self).__init__()
        self.gl = Sim3DEnvironment(self, width, height, lightness, False)
        self.gl.track.draw()
        '''VirtualCar(self, DriverDizzy(self.gl.track.start) if 1 else DriverBlind(TrackRef((NxpTrack.TRACK_DEF,
                                                                                          self.gl.track.start,
                                                                                          *tuple(itertools.chain(tuple((NxpTrack.deltas[x] for x in self.gl.track.path))*5))
                                                                                          ))
                                                                                , max_speed_m_per_s), str(0))'''
        # VirtualCar commented out because we want to use the TeamDriver instead of the default drivers. Keep for testing purposes.
        VirtualCar(self, TeamDriver(self.gl.track.start))
        self.slow_motion_scale = slow_motion_scale
        self.ref_adj = []
        self.normal_redraw_per = 1.0 / monitor_fps_min
        self.restart = False


    def simulation_speed_ctrl(self, pause=False):
        while True:
            for e in pygame.event.get():
                if e.type == QUIT:
                    sys.exit()
                elif e.type == KEYDOWN:
                    if e.key == K_ESCAPE:
                        pixy2sim.line_close()
                        sys.exit()
                    elif e.key == K_SPACE:
                        pause = not pause
                        print("simtime:", self.t) # helps to manual time laps in simulation time
                    elif K_0 <= e.key <= K_9:
                        self.slow_motion_scale = e.key - K_0
                    elif e.key == K_p: # take snapshots on disk
                        os.mkdir('snap')
                        fn = r'snap\snap_'
                        fn += datetime.datetime.utcnow().isoformat()[:23].replace(':', '_')+"UTC"
                        self.gl.imwrite(2, fn + '.png')
                        if pygame.key.get_mods() & KMOD_CTRL != 0:
                            self.gl.imwrite(1, fn + 't.png')
                        if self.gl.raster is not None and pygame.key.get_mods() & KMOD_SHIFT != 0:
                            cv2.imwrite(fn + 'o.png', np.flip(self.gl.raster, 0))
                    elif e.key == K_r:
                        self.restart = True
                    else:
                        Sim.keyboard_buffer.append(e.key)
                   
            if not pause:
                break
            pygame.time.wait(10) # unload CPU during pause

    def update_slowmotion(self, physics_dt, redraw_per, snap_div):
        slowmo_div0 = int(math.ceil(self.slow_motion_scale))
        slowmo_div = slowmo_div0 if slowmo_div0 != 0 else 1
        slowmo_per = redraw_per / slowmo_div
        phys_div = int(math.ceil(slowmo_per / physics_dt))
        phys_per = slowmo_per / phys_div
        print(phys_per, phys_div, slowmo_div, snap_div)
        cycles = phys_div * slowmo_div * snap_div
        return cycles, phys_div, phys_per, slowmo_div0

    def run(self, physics_dt=0.001, camera_fps=20, pause_on_start=True):
        # calibrate
        self.gl.sim_calibrate(self.components[0])
        if 0: # pause to inspect the calibration pattern (useful when repositioning camera to make sure pattern is still visible)
            self.simulation_speed_ctrl(pause_on_start)

        snap_per = 1.0 / camera_fps
        snap_div = int(math.ceil(snap_per / self.normal_redraw_per))
        redraw_per = snap_per / snap_div
        # calculate
        phys2observe_div, phys2draw_div, phys_per, slowmotion_div = self.update_slowmotion(physics_dt, redraw_per, snap_div)
        phys_sim_step_count = 0
        start = time.time()
        total_sleep_time_s = 0
        sim_time_offset = 0
        time_offset = 0
        last, fpsc, fpsa = time.time(), 0, 0 # virtual fps stat calculation variables
        while True:
            if divmod(phys_sim_step_count, phys2draw_div)[1] == 0 and \
                    not (self.t > 0 and slowmotion_div == 0 and self.components[0].motor.state <= 0):
                observe = divmod(phys_sim_step_count, phys2observe_div)[1] == 0
                if slowmotion_div != 0 or observe:
                    self.gl.draw_split_screen(self.components, observe,
                                              'press space to continue' if pause_on_start else None)
                    if 0: # calculate and display fps (in accelerated mode it will go over the monitor vertical refresh rate)
                        nt = time.time()
                        fps, last = 1.0/(nt-last), nt
                        fpsa += fps
                        fpsc +=1 
                        if fpsc >=100:
                            print(round(fpsa/fpsc))
                            fpsc = fpsa = 0
                self.simulation_speed_ctrl(pause_on_start)
                if pause_on_start:
                    start = time.time()
                    pause_on_start = False
                if observe and self.slow_motion_scale != slowmotion_div:
                    time_offset = time.time() - start
                    sim_time_offset = self.t
                    phys2observe_div, phys2draw_div, phys_per, slowmotion_div = self.update_slowmotion(physics_dt, redraw_per, snap_div)
                    phys_sim_step_count = 0
                inter_frame_sleep_time = (self.t - sim_time_offset) * slowmotion_div - (time.time() - start - time_offset)
                if slowmotion_div: # only apply sleep for realtime and slowmotion, not for accelerated simulation
                    sleep_ms = int(1000 * inter_frame_sleep_time)
                    pygame.time.wait(sleep_ms)
                    #print(ttsm, 'ms sleep')
                total_sleep_time_s += inter_frame_sleep_time if inter_frame_sleep_time > 0 else 0
            if not self.step(phys_per): # mechanical simulation step (increases simulation time and advances dynamical systems)
                break
            phys_sim_step_count += 1

        self.components[0].driver.finished()  # announce first driver that sim is done
        log.info('WallClock elapsed time:' + str(time.time() - start) + ' Simulated time: ' + str(self.t) + ' Sleep time: ' + str(total_sleep_time_s))
        self.simulation_speed_ctrl(True)





def main():
    CalibrationPattern.default_saturation_threshold = 38

    while True:
        pixy2sim.line_open()
        print('line_open')
        # Pixy 2 param dictionary used for setting up the simulator (This values can be overriden by loading a .prm Pixy 2 config file using the load_pixy_prm_file_to_dict function)
        kv = {
            # line detection params (standard)
            "Edge distance": 2,  # UINT16 g_dist
            "Edge threshold": 150,  # UINT16 g_thresh => g_hThresh
            "Minimum line width": 1,  # UINT16 g_minLineWidth
            "Maximum line width": 60,  # UINT16 g_maxLineWidth
            "Maximum merge distance": 1,  # UINT16 g_maxMergeDist (<4: reduced intersection detection!)
            "Minimum line length": 6,  # UINT16 g_minLineLength =>g_minLineLength2 (squared)
            "Maximum line compare": 1000, #7270,  # UINT32 g_maxLineCompare (frame to frame line tracking slack)
            "White line": 0,  # UINT8 g_whiteLine
            "Line filtering": 0,  # UINT8 g_lineFiltering`
            "Intersection filtering": 0,  # UINT8 g_primaryIntersection.setTiming(leading, trailing);
            "Manual vector select": 0,  # UINT8 g_manualVectorSelect

            # params not available through standard Pixy2 API, but manually exposed for research (50, 60, 60, 8 => better segments, but requires custom fw...)
            #"g_maxSegTanAngle": int(math.tan(math.pi / 4) * 1000), # g_maxSegTanAngle (angle between two segments under which are considered single segment)
            #"g_maxEquivTanAngle": int(math.tan(math.pi / 10) * 1000),  #
            #"g_maxTrackingTanAngle": int(math.tan(math.pi / 10) * 1000),
            #"g_pointsPerSeg": 12,

            # extra commands (not params, but part of the Pixy2 API)
            # "line_setVector": 0, #UINT8 (activates manual vector select and sets the line index to become main vector)
            # "line_reversePrimary": 0 #UINT8 (parameter is ignored, sending the setting will cause Pixy API call to reverse vector orientation)

            # unsupported line detection params (standard on Pixy2, but not supported in this version of the simulator)
            # "Barcode filtering": 1, #UINT8 g_barcodeFiltering (barcode not supported)
            # "Default turn angle": 0, #UINT16
            # "Delayed turn": 0, #UINT8 g_delayedTurn
        }

        # Utility to use .prm file for overriding config dictionary values
        def load_pixy_prm_file_to_dict(fpath="pixy0.prm", kv_init=None):
            kv_init = kv_init or kv  # workaround single time default param initialization
            from io import open
            from re import match
            with open(fpath, "r") as cfg:
                for ln in cfg.readlines():
                    m = match(r'\s*<data value\s*=\s*"(?P<v>\d+)".+key\s*=\s*"(?P<k>[^"]*)"/>', ln)
                    if m:
                        k = m.group("k")
                        if k in kv_init:
                            v = int(m.group("v"))
                            if v != kv_init[k]:
                                print("Overiding", k, 'from', kv_init[k], 'to', v, 'based on', fpath, 'config file')
                                kv_init[k] = v

        #load_pixy_prm_file_to_dict() #uncomment to use config file for init override

        if 1:  # send the settings from the config dict to the simulator
            for k, v in kv.items():
                if pixy2sim.set_pixy2_param(k, v): print('FAILED to set', k, 'to', v)

        cs = Sim(monitor_fps_min=10
                 , slow_motion_scale=0.0
                 , width=636
                 , height = 416
                 , lightness=0.75)
        cs.run(physics_dt=0.0010
               , camera_fps=60)
        pixy2sim.line_close()
        print('line_close')

main()
