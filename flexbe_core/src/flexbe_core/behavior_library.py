#!/usr/bin/env python
import os
import rospy
from rospkg import RosPack
import xml.etree.ElementTree as ET
import zlib


'''
Created on 10.01.2017

@author: Philipp Schillinger
'''

class BehaviorLibrary(object):
	'''
	Provides a list of all known behavior manifests.
	'''

	def __init__(self):
		self._rp = RosPack()
		self._behavior_lib = dict()
		self.parse_packages()


	def parse_packages(self):
		"""
		Parses all ROS packages to update the internal behavior library.
		"""
		self._behavior_lib = dict()
		for pkg in self._rp.list():
			for export in self._rp._load_manifest(pkg).exports:
				if export.tag == "flexbe_behaviors":
					self._add_behavior_manifests(self._rp.get_path(pkg), pkg)


	def _add_behavior_manifests(self, path, pkg=None):
		"""
		Recursively add all behavior manifests in the given folder to the internal library.
		If a package name is specified, only manifests referring to this package are added.

		@type path: string
		@param path: Path of the folder to be traversed.
		
		@type pkg: string
		@param pkg: Optional name of a package to only add manifests referring to this package.
		"""
		for entry in os.listdir(path):
			entry_path = os.path.join(path, entry)
			if os.path.isdir(entry_path):
				self._add_behavior_manifests(entry_path, pkg)
			elif entry.endswith(".xml") and not entry.startswith("#"):
				m = ET.parse(entry_path).getroot()
				# structure sanity check
				if m.tag != "behavior" \
				or len(m.findall(".//executable")) == 0 \
				or m.find("executable").get("package_path") is None \
				or len(m.find("executable").get("package_path").split(".")) != 2:
					continue;
				e = m.find("executable")
				if pkg is not None and e.get("package_path").split(".")[0] != pkg:
					continue # ignore if manifest not in specified package
				be_id = zlib.adler32(e.get("package_path"))
				self._behavior_lib[be_id] = {
					"name": m.get("name"),
					"package": e.get("package_path").split(".")[0],
					"file": e.get("package_path").split(".")[1],
					"class": e.get("class")
				}


	def get_behavior(self, be_id):
		"""
		Provides the library entry corresponding to the given ID.

		@type be_id: int
		@param be_id: Behavior ID to look up.

		@return Corresponding library entry or None if not found.
		"""
		try:
			return self._behavior_lib[be_id]
		except KeyError:
			rospy.logwarn("Did not find ID %d in libary, updating..." % be_id)
			self.parse_packages()
			return self._behavior_lib.get(be_id, None)


	def find_behavior(self, be_name):
		"""
		Searches for a behavior with the given name and returns it along with its ID.

		@type be_name: string
		@param be_name: Behavior ID to look up.

		@return Tuple (be_id, be_entry) corresponding to the name or (None, None) if not found.
		"""
		find = lambda: next((id, be) for (id, be) in self._behavior_lib.items() if be["name"] == be_name)
		try:
			return find() 
		except StopIteration:
			rospy.logwarn("Did not find behavior '%s' in libary, updating..." % be_name)
			self.parse_packages()
			return find()


	def count_behaviors(self):
		"""
		Counts the available behaviors.

		@return Number of behaviors.
		"""
		return len(self._behavior_lib)






















