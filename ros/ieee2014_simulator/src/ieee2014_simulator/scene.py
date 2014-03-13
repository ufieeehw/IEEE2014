from __future__ import division

import math

import numpy
from tf import transformations

from ieee2014_mission_runner import fine_targeting, render

INCH = fine_targeting.INCH

def draw_scene(r):
	course_length = (97 - 3/4 * 2) * INCH
	course_width = (49 - 3/4 * 2) * INCH

	'''r.draw_polygon([
		(+course_length/2, +course_width/2, 0),
		(-course_length/2, +course_width/2, 0),
		(-course_length/2, -course_width/2, 0),
		(+course_length/2, -course_width/2, 0),
	], (0, 0, 0, 255))'''
	def lerp(x, (a, b)):
		return (1-x)*a + (x)*b
	x_range = -course_length/2, +course_length/2
	y_range = -course_width/2, +course_width/2
	for i in xrange(3):
		for j in xrange(3):
			sub_x_range = lerp(i/3, x_range), lerp((i+1)/3, x_range)
			sub_y_range = lerp(j/3, y_range), lerp((j+1)/3, y_range)
			r.draw_polygon([
				(sub_x_range[1], sub_y_range[1], 0),
				(sub_x_range[0], sub_y_range[1], 0),
				(sub_x_range[0], sub_y_range[0], 0),
				(sub_x_range[1], sub_y_range[0], 0),
			], (0, 0, 0, 255))
	
	def line(start, end, width=3/4*INCH, color=(255, 255, 255, 255)):
		start = numpy.array(start)
		threeify = lambda (x, y): (x, y, 0)
		perp = numpy.array([
			[0, -1],
			[1, 0],
		]).dot(transformations.unit_vector(end-start)) * width/2
		r.draw_polygon([
			threeify(start+perp),
			threeify(end+perp),
			threeify(end-perp),
			threeify(start-perp),
		], color)
	
	def cvtLogical_to_Forrest((x,y)):
		return x - course_length/2, y - course_width/2
	
	line(cvtLogical_to_Forrest((35.25*INCH, 0)), cvtLogical_to_Forrest((35.25*INCH, course_width)))
	line(cvtLogical_to_Forrest((57.75*INCH, 0)), cvtLogical_to_Forrest((57.75*INCH, course_width)))
	
	radius = 49*INCH
	angles = numpy.linspace(-math.asin(course_width/2 / radius), math.asin(course_width/2 / radius), 10)
	center = numpy.array([-course_length/2, 0])
	cossin = lambda angle: numpy.array([math.cos(angle), math.sin(angle)])
	for a, b in zip(angles[:-1], angles[1:]):
		line(center + radius*cossin(a), center + radius*cossin(b))
	
	line(cvtLogical_to_Forrest((12*INCH, 9.88*INCH)), cvtLogical_to_Forrest((course_length,9.88*INCH))),
	
	# blocks!
	
	fine_targeting.draw_target(r)
