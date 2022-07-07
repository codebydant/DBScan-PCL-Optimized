#!/usr/bin/env python
import re
import os
import sys

def gen_op_families():
	op_families = {}
	with open("gas.vim") as lines:
		for line in lines:
			m = re.match(r"syn\s+keyword\s+gasOpcode_(\S+)", line)
			if m != None:
				toks = re.split("\s+", line.strip())
				for i in range(3, len(toks)):
					op_families[toks[i]] = m.group(1)
	return op_families


def get_instructions(filename):
	global invalid
	instructions_used = set()

	with os.popen('objdump -d ' + filename) as pipe:
		for line in pipe:
			line = line.strip()
			m = re.match(r"\S+:\s+(\S\S\s)*\S\S\s\s+(\S+)", line.strip())
			if m != None:
				iname = m.group(2)
				instructions_used.add(iname)
	return instructions_used


def get_families_used(families, ops):
	families_used = set()
	missing_family = set()

	for op in ops:
		try:
			families_used.add(families[op])
		except KeyError as e:
			missing_family.add(op)

	return (families_used, missing_family)

def main():
	if len(sys.argv) < 2 or len(sys.argv) > 2:
		print("Usage: ./binary_families.py path-to-binary")
		sys.exit(0)
	
	filename = sys.argv[1]
	instructions_used = get_instructions(filename)
	
	op_families = gen_op_families()
	families_used, missing_family = get_families_used(op_families, instructions_used)

	used = sorted(list(families_used))
	missing_family = sorted(list(missing_family))
	print("These instruction families were used:")
	print(", ".join(used))
	
	print("These instructions could not be categorized:")
	print(", ".join(missing_family))

main()