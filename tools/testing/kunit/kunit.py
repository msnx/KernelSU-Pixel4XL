#!/usr/bin/python3
# SPDX-License-Identifier: GPL-2.0

# A thin wrapper on top of the KUnit Kernel

import argparse
import sys
import os
import time

import kunit_config
import kunit_kernel
import kunit_parser

parser = argparse.ArgumentParser(description='Runs KUnit tests.')

parser.add_argument('--raw_output', help='don\'t format output from kernel',
		    action='store_true')

parser.add_argument('--timeout', help='maximum number of seconds to allow for '
		    'all tests to run. This does not include time taken to '
		    'build the tests.', type=int, default=300,
		    metavar='timeout')

cli_args = parser.parse_args()
linux = kunit_kernel.LinuxSourceTree()

config_start = time.time()
success = linux.build_reconfig()
config_end = time.time()
if not success:
	quit()

kunit_parser.print_with_timestamp('Building KUnit Kernel ...')

build_start = time.time()
success = linux.build_um_kernel()
build_end = time.time()
if not success:
	quit()

kunit_parser.print_with_timestamp('Starting KUnit Kernel ...')
test_start = time.time()

if cli_args.raw_output:
	kunit_parser.raw_output(linux.run_kernel(timeout=cli_args.timeout))
else:
	kunit_parser.parse_run_tests(linux.run_kernel(timeout=cli_args.timeout))

test_end = time.time()

kunit_parser.print_with_timestamp((
	"Elapsed time: %.3fs total, %.3fs configuring, %.3fs " +
	"building, %.3fs running.\n") % (test_end - config_start,
	config_end - config_start, build_end - build_start,
	test_end - test_start))
