#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Setup for Lucy
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

from distutils.core import setup

setup(
        name='configuration',
        version='0.3',
        packages=['', 'tests', 'tests.errors', 'tests.parser', 'tests.datatypes', 'tests.simulator',
                  'tests.simulator.errors', 'tests.simulator.datatypes', 'tests.configuration',
                  'tests.genetic_operators', 'errors', 'parser', 'datatypes', 'simulator', 'simulator.errors',
                  'simulator.datatypes', 'configuration', 'genetic_operators'],
        url='',
        license='GNU/GPLV3',
        author='Andrés Aguirre Dorelo',
        author_email='aaguirre@fing.edu.uy',
        description='Lucy Installer'
)
