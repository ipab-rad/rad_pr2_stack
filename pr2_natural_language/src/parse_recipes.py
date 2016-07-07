# Parsing natural language commands related to a kitchen setting, to be carried out by a PR2.
# Uses the Carnegie Mellon University Recipe Database (CURD): http://www.cs.cmu.edu/~ark/CURD/
# June - July 2016
# Stanford University x University of Edinburgh Bing Scholar Programme
# Jessica Zhao

import numpy as np
import re
import nltk
from nltk.stem.porter import *
from nltk.corpus import wordnet as wn
import random
import csv
import os
import sys
import collections
import operator
import itertools
from operator import and_
from functools import reduce
from os import listdir
from os.path import isfile, join

RECIPES_PATH = "/home/alex/pr2_ws/src/annotated_recipes/"
MEASURE_WORDS = {"cup", "cups", "spoon", "spoons", "teaspoon", "teaspoons", "tablespoon", "tablespoons", "pinch", "inch", "cm", "ounce", "pound"}
DELIM = ",| and | or "
YES_WORDS = {"egg", "eggs", "electric", "nonstick", "pan", "oven", "white", "red", "brown", "set"}
CAT = {"ACTION", "TOOL", "TARGET"}

# Graphical representation of a kitchen
class KitchenGraph:
	def __init__(self):
		# item --> CATEGORY (ACTION, TARGET, or TOOL)
		self.nodes = collections.defaultdict(str)

		# edge (A,B) --> frequency weight
		self.edges = collections.defaultdict(int)

		# node --> list of neighbors
		self.neighbors = collections.defaultdict(set)

		# original text --> parsing
		self.textToAnnotation = collections.defaultdict(list)

	# Updating the nodes, edges, and neighbors with newly learned entities and relations
	def update_graph(self, new_annotation, og_text):
		# Updating nodes
		temp_nodes = set()
		for key, value in new_annotation.iteritems():
			new_annotation[key] = value.replace("\"", "") # Stripping quotes from parsed data
			if key is not "PRODUCT":
				for ing in new_annotation[key].split(","):
					ing = ing.strip()
					if ing is not "":
						if ing in self.nodes and key is not self.nodes[ing]: #same node, different labels
							ing += " (" + key + ")"
						temp_nodes.add(ing)
						self.nodes[ing] = key

		# Updating neighbors
		for node in temp_nodes:
			if node is not "":
				self.neighbors[node] |= temp_nodes.difference(node)

		# Updating edges -- order pair lexicographically
		for pair in list(itertools.combinations(temp_nodes, 2)):
			if "" in pair: continue
			pair = (min(pair), max(pair))
			self.edges[pair] += 1

		# Keep track of original text mapping to this parsing
		self.textToAnnotation[og_text].append(new_annotation)

	# Exports data into nodes.csv and edges.csv
	def export_csv(self):
		out_nodes = csv.writer(open("nodes.csv","w"), delimiter=',',quoting=csv.QUOTE_ALL)
		node_fields = ["Id", "Label"]
		out_nodes.writerow(node_fields)
		for node, label in self.nodes.iteritems():
			out_nodes.writerow((node, label))
		out_edges = csv.writer(open("edges.csv","w"), delimiter=',',quoting=csv.QUOTE_ALL)
		edge_fields = ["Source", "Target", "Weight", "Type"]
		out_edges.writerow(edge_fields)
		for edge, weight in self.edges.iteritems():
			if edge[0] != "" and edge[1] != "":
				out_edges.writerow((edge[0], edge[1], str(weight), "Undirected"))

	# Outputs useful info about the state of the graph
	def to_string(self):

		# General data
		out = ""
		out += "NUM NODES: " + str(len(self.nodes)) + "\n"
		out += "NUM EDGES: " + str(len(self.edges)) + "\n\n"

		# Top weights
		out += "15 MOST FREQUENT EDGES:\n"
		sorted_edges = sorted(self.edges, key=self.edges.get, reverse=True)
		for i in range(1, 16):
			out += str(i) + ") " + str(sorted_edges[i]) + ": " + str(self.edges[sorted_edges[i]]) + "\n"
		out += "\n"

		# Sample queries --> parsed
		out += "10 SAMPLE TEXT --> ANNOTATION:\n"
		for i in range(1, 11):
			text = random.choice(self.textToAnnotation.keys())
			out += "TEXT: " + text + "\n"
			for rule in self.textToAnnotation[text]:
				for key, val in rule.iteritems():
					out += str(key) + ": " + val + "\n"
			out += "\n"
		return out

# Remove extraneous descriptors from annotations - numbers, adjectives
# Uses NLTK tagger and also prunes from a common measure words list
def simplify_parsed(parsed_annotation):
	new_parsing = collections.defaultdict()
	for key, val in parsed_annotation.iteritems():
		outs = []
		split_val = re.split(DELIM, val)
		for phrase in split_val:
			tokens = nltk.word_tokenize(phrase)
			tagged = nltk.pos_tag(tokens)
			# Preserve if only 2 or fewer words
			if len(tagged) > 2:
				out_list = []
				for pair in tagged:
					if pair[0] in YES_WORDS:
						out_list.append(pair[0])
					elif pair[0] not in MEASURE_WORDS:
						if pair[1] == "NN" or pair[1] == "NNS":
							out_list.append(pair[0])
				outs.append(' '.join(out_list))
			else:
				outs.append(phrase)
		output = ", ".join(outs)
		if output is not "": new_parsing[key] = output.lower()
		else: new_parsing[key] = val.lower()

		# if len(outs) > 0: new_parsing[key] = ", ".join(outs)
		# else: new_parsing[key] = val

	# print "OLD: ", parsed_annotation
	# print "NEW: ", new_parsing
	# print "\n"

	return new_parsing

# Combine involves ingredients in { }, thus we deal with args instead of args_split
# FORM: combine(ingredient set ins, ingredient out, string outdesc, string manner)
# EXAMPLE: combine({ing1, ing2, ing3}, ing9, "cheeses", "")
# string manner can be empty
def combine(args, recipe_space, parsed_annotation):
	index = args.find("}", 0, len(args))
	ins = ""
	for ing in args[1:index].split(","):
		ing = ing.strip()
		if ing in recipe_space:
			if ins != "": ins += ", "
			ins += recipe_space[ing]
	args = [arg.strip() for arg in args[index+2:].split(",")]
	parsed_annotation["TARGET"] = ins
	parsed_annotation["ACTION"] = "combine"
	parsed_annotation["PRODUCT"] = args[1]
	#if args[2] is not "": parsed_annotation["COMMENT"] = args[2]
	recipe_space[args[0]] = args[1] # Updating recipe space with new product

# 0 FORM: mix(ingredient in, tool t, ingredient out, string outdesc, string manner)
# 0 EXAMPLE: mix(ing10, t1, ing11, "beat mixture", "beat")
# 1 FORM: cut(ingredient in, tool t, ingredient out, string outdesc, string manner)
# 1 EXAMPLE: cut(ing0, , ing8, "French bread slices with pockets", "cut a pocket")
# 2 FORM: cook(ingredient in, tool t, ingredient out, string outdesc, string manner)
# 2 EXAMPLE: cook(ing16, t0, ing17, "fragrant spices", "cook until fragrant, 2 minutes")
# tool t and string manner can be empty
def mix_cut_cook(args, recipe_space, parsed_annotation, action):
	if args[0] in recipe_space: parsed_annotation["TARGET"] = recipe_space[args[0]]
	if args[1] is not "" and args[1] in recipe_space: parsed_annotation["TOOL"] = recipe_space[args[1]]
	parsed_annotation["PRODUCT"] = args[3]
	parsed_annotation["ACTION"] = action
	recipe_space[args[2]] = args[3]

# FORM: set(tool t, string setting)
# EXAMPLE: set(t2, "medium-high heat")
# neither can be empty
def set_(args, recipe_space, parsed_annotation):
	parsed_annotation["TARGET"] = recipe_space[args[0]]
	parsed_annotation["ACTION"] = "set " + args[1]

# FORM: put(ingredient i, tool t)
# EXAMPLE: put(ing16, t3)
# neither can be empty
def put(args, recipe_space, parsed_annotation):
	if args[0] in recipe_space: parsed_annotation["TARGET"] = recipe_space[args[0]]
	if args[1] in recipe_space: parsed_annotation["TOOL"] = recipe_space[args[1]]
	parsed_annotation["ACTION"] = "put"

# FORM: do(ingredient in, tool t, ingredient out, string outdesc, string manner)
# EXAMPLE: do(ing8, , ing18, "French bread slices with open pockets", "open pockets")
# tool t and string manner can be empty
def do(args, recipe_space, parsed_annotation):
	if args[0] in recipe_space: parsed_annotation["TARGET"] = recipe_space[args[0]]
	if args[1] is not "" and args[1] in recipe_space: parsed_annotation["TOOL"] = recipe_space[args[1]]
	if len(args) > 3: recipe_space[args[2]] = args[3]
	if len(args) > 4: parsed_annotation["ACTION"] = args[4]
	else: parsed_annotation["ACTION"] = "do"

# FORM: separate(ingredient in, ingredient out1, string out1desc, ingredient out2, string out2name, string manner)
# EXAMPLE: separate(ing0, ing1, "raisins", ing2, "peanuts and almonds", "")
# string manner can be empty
def separate(args, recipe_space, parsed_annotation):
	if args[0] in recipe_space: parsed_annotation["TARGET"] = recipe_space[args[0]]
	recipe_space[args[1]] = args[2]
	recipe_space[args[3]] = args[4]
	if args[5] is not "": parsed_annotation["ACTION"] = args[5]
	else: parsed_annotation["ACTION"] = "separate"

# FORM: serve(ingredient ing, string manner)
# manner can be null
def serve(args, recipe_space, parsed_annotation):
	if args[0] in recipe_space: parsed_annotation["TARGET"] = recipe_space[args[0]]
	parsed_annotation["ACTION"] = "serve"
	if args[1] is not "": parsed_annotation["ACTION"] += " " + args[1]

# Given a text instruction, output a mapping of (ACTION --> x, TARGET --> y, TOOL --> z)
def parse_annotation(full_line, recipe_space, graph):

	# Separating original and annotation
	full_line = full_line[0]
	og_text = re.findall('<originaltext>(.*)</originaltext>', full_line)[0]
	annotation = re.findall('<annotation>(.*)</annotation>', full_line)[0]

	# Structuring before parsing
	parsed_annotation = collections.defaultdict()
	parsed = re.findall('(^[^\(]*)\((.*)\)', annotation)[0]
	rule = parsed[0]
	args = parsed[1]
	args_split = [arg.strip().replace("{", "").replace("}", "") for arg in args.split(",")]

	# Parsing
	if rule == "create_ing" or rule == "create_tool":
		recipe_space[args_split[0]] = args_split[1]
	else:
		if rule == "combine":
			combine(args, recipe_space, parsed_annotation)
		elif rule == "cut" or rule == "mix" or rule == "cook":
			mix_cut_cook(args_split, recipe_space, parsed_annotation, rule)
		elif rule == "set":
			set_(args_split, recipe_space, parsed_annotation)
		elif rule == "put":
			put(args_split, recipe_space, parsed_annotation)
		elif rule == "do":
			do(args_split, recipe_space, parsed_annotation)
		elif rule == "separate":
			separate(args_split, recipe_space, parsed_annotation)
		elif rule == "serve":
			serve(args_split, recipe_space, parsed_annotation)
		new_parsing = simplify_parsed(parsed_annotation)
		graph.update_graph(new_parsing, og_text)


# For each recipe, extract parsed instructions for each line
def process_recipes(graph):
	all_files = [join(RECIPES_PATH, f) for f in listdir(RECIPES_PATH) if isfile(join(RECIPES_PATH, f))]
	for recipe in all_files:
		recipe_space = collections.defaultdict() # Tracks all ingredients, tools in current recipe
		lines = open(recipe).readlines()
		for line in lines:
			full_line = re.findall('<line>(.*)</line>', line)
			if full_line:
				parse_annotation(full_line, recipe_space, graph)


# Updates query space with most recent items mentioned
def update_query_space(annotation, query_space):
	for key, val in annotation.iteritems():
		query_space[key].insert(0, val)

# Fetches most likely tool for common actions
def get_tool_from_action(action, graph):

	tools = [neighbor for neighbor in graph.neighbors[action] if graph.nodes[neighbor] == "TOOL"]
	tool_map = collections.defaultdict()
	for tool in tools:
		edge = (min(action, tool), max(action, tool))
		tool_map[tool] = graph.edges[edge]
	sorted_tools = sorted(tool_map, key=tool_map.get, reverse=True)
	if len(sorted_tools) > 0: return sorted_tools[0]
	return None

# Given a string query, return the most likely (ACTION, TOOL, TARGET)
def parse_query(query, graph, query_space):

	# (0) PRE-PROCESSING
	query = query.lower()
	tokens = nltk.word_tokenize(query)
	tagged = nltk.pos_tag(tokens)
	potential_actions = []

	# Find key words from query
	phrases = []
	phrase_tokens = []
	foundNN = False
	good_phrases = []
	prior = ""
	confidences = collections.defaultdict(int) # ACTION, TOOL, TARGET --> confidence 0 or 1

	for pair in tagged:
		if pair[1] == "VB": # found potential action
			graph.nodes[pair[0]] = "ACTION"
			potential_actions.append(pair[0])
		# if pair[0] in food: # found potential food
		# 	graph.nodes[pair[0]] = "TARGET"
		if prior is not "" and (prior + " " + pair[0]) in graph.nodes:
			good_phrases.append(prior + " " + pair[0])
		elif pair[0] in graph.nodes:
			good_phrases.append(pair[0])
		elif pair[1] == "NN" or pair[1] == "NNS":
			phrase_tokens.append(pair[0])
		elif len(phrase_tokens) > 0: # phrase finished
			phrases.append(' '.join(phrase_tokens))
			phrase_tokens = []
		prior = pair[0]

	# (1) PLAN A: all 3 categories appear in query
	good_phrases += [phrase for phrase in phrases if phrase in graph.nodes]
	pre_candidates = collections.defaultdict(set)
	for phrase in good_phrases:
		pre_candidates[graph.nodes[phrase]].add(phrase)
		confidences[graph.nodes[phrase]] += 1

	# Real-time learning: adds edges between phrases found in graph
	for pair in list(itertools.combinations(good_phrases, 2)):
		if "" in pair: continue
		pair = (min(pair), max(pair))
		graph.edges[pair] += 1
		graph.neighbors[pair[0]].add(pair[1])
		graph.neighbors[pair[1]].add(pair[0])

	cand = collections.defaultdict()
	for key, val in pre_candidates.iteritems():
		cand[key] = ', '.join(val)

	# Found all categories
	missing_cat = list(CAT.difference(cand.keys()))
	if len(cand) == 3: return (cand, confidences)

	# (2) PLAN B: Otherwise, try to guess other categories
	if len(cand) == 2: # Missing 1 category
		items = set()
		for val in cand.values(): # Splitting in case of multiple targets, usually
			if ',' in val:
				for item in val.split(', '): items.add(item)
			else: items.add(val)
		overlap = set(neighbor for neighbor in graph.neighbors[list(items)[0]] if graph.nodes[neighbor] == missing_cat[0])
		for item in items:
			item_set = set(neighbor for neighbor in graph.neighbors[item] if graph.nodes[neighbor] == missing_cat[0])
			overlap = overlap.intersection(item_set)

		if len(overlap) > 0:
			overlap_map = collections.defaultdict(int)
			for node in overlap: # Rank overlapping neighbors
				for item in items:
					edge = (min(node, item), max(node, item))
					overlap_map[node] += graph.edges[edge]
			sorted_possibilities = sorted(overlap_map, key=overlap_map.get, reverse=True)

			# print "TOP SUGGESTIONS"
			# for sp in sorted_possibilities:
			# 	print sp, overlap_map[sp]

			cand[missing_cat[0]] = sorted_possibilities[0]
			confidences[missing_cat[0]] = 0
			return (cand, confidences)

	# (3) PLAN C: Check query_space for likely candidates
	# Filter likelihood by at least 1 edge weight observed
	if len(cand) == 0: return None
	found = cand.values()
	for cat in missing_cat:
		if cat in query_space:
			last_item = query_space[cat][0]
			matches = 0
			for item in found:
				edge = (min(item, last_item), max(item, last_item))
				if graph.edges[edge] > 0: matches += 1
			if matches >= len(found): cand[cat] = last_item
	if "ACTION" in cand and "TOOL" not in cand:
		tool = get_tool_from_action(cand["ACTION"], graph)
		if tool is not None: cand["TOOL"] = tool
		confidences["TOOL"] = 0
	return (cand, confidences)

# Cleans text if there are extranous parens
def clean_candidate(cand):
	for key, val in cand.iteritems():
		if "(" in val: cand[key] = val[:val.index("(")-1]

# Outputs script for PR2
def PR2_speech(query_result):
	if query_result is None: return "I'm a bit at a loss here, sorry about that! Care to elaborate?"
	parsed = query_result[0]
	confidences = query_result[1]
	clean_candidate(parsed)

	out = ""
	if "ACTION" in parsed:

		# IF ACTION CONFIDENCE IS 0!!!! ADD something here

		if "TARGET" in parsed:
			if confidences["TARGET"] == 0:
				out += "I'm assuming you're talking about the " + parsed["TARGET"] + "? I can " + parsed["ACTION"] + " those! "
			else:
				options = ["You give great instructions! I can definitely help you " + parsed["ACTION"] + " the " + parsed["TARGET"] + ". ",
				"I've got it, " + parsed["ACTION"] + " the " + parsed["TARGET"] + ", coming up! ", "Ooh, I'd love to " + parsed["ACTION"] + " the " + parsed["TARGET"] + "! "]
				out += random.choice(options)
		else:
			out += "I see you want to " + parsed["ACTION"] + " something, but I'm not sure what. "

		if "TOOL" in parsed:
			if confidences["TOOL"] == 0:
				options = ["Might I suggest we use a " + parsed["TOOL"] + " for that? ", "I think a " + parsed["TOOL"] + " would be handy here!"]
				out += random.choice(options)
			else:
				out += "We'll use the " + parsed["TOOL"] + " as you suggested. "
	else:
		if "TARGET" in parsed:
			out += "Ah, the " + parsed["TARGET"] + ". I'm not sure what you want me to do with them, though. "
		else: # TARGET and ACTION are empty
			if "TOOL" in parsed:
				out += "Ah, I'm not sure I caught any ingredients in there. "
				if confidences["TOOL"] == 0:
					out += "However, based on what I've heard so far, I think a " + parsed["TOOL"] + " may be helpful for this scenario. "
				else:
					out += "I do understand your desire to use a " + parsed["TOOL"] + ", and we definitely shall! You'll just have to clarify the rest for me. "
			else: # TARGET and ACTION and TOOL are empty
				out += "I'm a bit at a loss here, sorry about that! Care to elaborate?"
	return out


# Get list of objects PR2 can look at
def getObjects(parsed):
	objects = []
	for key, val in parsed.iteritems():
		if key is not "ACTION":
			objects += val.split(", ")
	return objects

# Given a single query, outputs a tuple of (PR2's speech, list of objects)
def single_query_to_speech(query, graph):
	stemmer = PorterStemmer()
	query_space = collections.defaultdict(list)
	result = parse_query(query, graph, query_space)
	if result is None: # Try porter stemmer
		words = [stemmer.stem(word) for word in query.split(" ")]
		stemmed_query = ' '.join(words)
		result = parse_query(stemmed_query, graph, query_space)
		if result is None: return (PR2_speech(result), [])
	return (PR2_speech(result), getObjects(result[0]))

# Builds graph and processes recipes
def initializeGraph():
	graph = KitchenGraph()
	process_recipes(graph)
	return graph

def main():

	# SINGLE QUERY EXAMPLE:
	# graph = initializeGraph()
	# query = "mix the vegetables and the carrots"
	# single_query_to_speech(query, graph)
	# sys.exit()

	graph = KitchenGraph()
	process_recipes(graph)
	stemmer = PorterStemmer()

	print graph.to_string()

	# Query model
	# Within recipe space, keeps stack of previously mentioned items
	query_space = collections.defaultdict(list)
	while True:
		query = raw_input("Enter a cooking instruction, or 0 to exit: ")
		if query == "0": break
		result = parse_query(query, graph, query_space)

		if result is None: # Try porter stemmer
			words = [stemmer.stem(word) for word in query.split(" ")]
			stemmed_query = ' '.join(words)
			result = parse_query(stemmed_query, graph, query_space)

		print "\nPR2 speech:"
		print PR2_speech(result)
		if result is not None:
			parsed = result[0]
			confidences = result[1]
			out = ""
			for key, val in parsed.iteritems():
				out += key + ": " + val + "\n"
			update_query_space(parsed, query_space)
			print "\nCategory breakdown:\n", out

			# print "QUERY SPACE: ", query_space

if __name__ == "__main__":
    main()


# TO DO ~~~~~~~~~~

# SEND LIST of TOOLS / April tag items!!

# run some tests / train and test sets

# what happens when item has multiple labels (can be tool, target, etc. in different contexts???)

# FUTURE WORK ~~~~~~~~

# min edit distance?? spell check incorporation, STEMMING
# animation / video of most likely connections

# SEMANTIC MODELS: ACTION blah blah TARGET blah blah TOOL and find closest matches with future phrases

# LIST OF FOODS:
# food = wn.synset('food.n.02')
# food_list = list(set([w for s in food.closure(lambda s:s.hyponyms()) for w in s.lemma_names]))

# FUTURE WORK: probabilistic model when guessting missing categories -- based on database & what user says
# future work section: PR2 can ask clarifying questions!


