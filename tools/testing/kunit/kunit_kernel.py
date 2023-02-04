# SPDX-License-Identifier: GPL-2.0

import logging
import subprocess
import os

import kunit_config

KCONFIG_PATH = '.config'

class ConfigError(Exception):
	"""Represents an error trying to configure the Linux kernel."""


class BuildError(Exception):
	"""Represents an error trying to build the Linux kernel."""


class LinuxSourceTreeOperations(object):
	"""An abstraction over command line operations performed on a source tree."""

	def make_mrproper(self):
		try:
			subprocess.check_output(['make', 'mrproper'])
		except OSError as e:
			raise ConfigError('Could not call make command: ' + e)
		except subprocess.CalledProcessError as e:
			raise ConfigError(e.output)

	def make_olddefconfig(self):
		try:
			subprocess.check_output(['make', 'ARCH=um', 'olddefconfig'])
		except OSError as e:
			raise ConfigError('Could not call make command: ' + e)
		except subprocess.CalledProcessError as e:
			raise ConfigError(e.output)

	def make(self):
		try:
			subprocess.check_output(['make', 'ARCH=um'])
		except OSError as e:
			raise BuildError('Could not call execute make: ' + e)
		except subprocess.CalledProcessError as e:
			raise BuildError(e.output)

	def linux_bin(self, params, timeout):
		"""Runs the Linux UML binary. Must be named 'linux'."""
		process = subprocess.Popen(
			['./linux'] + params,
			stdin=subprocess.PIPE,
			stdout=subprocess.PIPE,
			stderr=subprocess.PIPE)
		process.wait(timeout=timeout)
		return process


class LinuxSourceTree(object):
	"""Represents a Linux kernel source tree with KUnit tests."""

	def __init__(self):
		self._kconfig = kunit_config.Kconfig()
		self._kconfig.read_from_file('kunitconfig')
		self._ops = LinuxSourceTreeOperations()

	def clean(self):
		try:
			self._ops.make_mrproper()
		except ConfigError as e:
			logging.error(e)
			return False
		return True

	def build_config(self):
		self._kconfig.write_to_file(KCONFIG_PATH)
		try:
			self._ops.make_olddefconfig()
		except ConfigError as e:
			logging.error(e)
			return False
		validated_kconfig = kunit_config.Kconfig()
		validated_kconfig.read_from_file(KCONFIG_PATH)
		if not self._kconfig.is_subset_of(validated_kconfig):
			logging.error('Provided Kconfig is not contained in validated .config!')
			return False
		return True

	def build_reconfig(self):
		"""Creates a new .config if it is not a subset of the kunitconfig."""
		if os.path.exists(KCONFIG_PATH):
			existing_kconfig = kunit_config.Kconfig()
			existing_kconfig.read_from_file(KCONFIG_PATH)
			if not self._kconfig.is_subset_of(existing_kconfig):
				print('Regenerating .config ...')
				os.remove(KCONFIG_PATH)
				return self.build_config()
			else:
				return True
		else:
			print('Generating .config ...')
			return self.build_config()

	def build_um_kernel(self):
		try:
			self._ops.make_olddefconfig()
			self._ops.make()
		except (ConfigError, BuildError) as e:
			logging.error(e)
			return False
		used_kconfig = kunit_config.Kconfig()
		used_kconfig.read_from_file(KCONFIG_PATH)
		if not self._kconfig.is_subset_of(used_kconfig):
			logging.error('Provided Kconfig is not contained in final config!')
			return False
		return True

	def run_kernel(self, args=[], timeout=None):
		args.extend(['mem=256M'])
		process = self._ops.linux_bin(args, timeout)
		with open('test.log', 'w') as f:
			for line in process.stdout:
				f.write(line.rstrip().decode('ascii') + '\n')
				yield line.rstrip().decode('ascii')
