import json

from node import Node

def getProblem(filename):
    with open(filename) as problem_file:
        problem = json.load(problem_file)
    return problem

def graphToJson(graph, filename):
    with open(filename, 'w') as solution_file:
        data = {"defenders": []}
        encoder = OffenserEncoder()
        for node in graph:
            data["defenders"].append(encoder.encode(node))
        json.dump(data, solution_file)

class OffenserEncoder(json.JSONEncoder):
    def default(self, offenser):
        return [offenser[0], offenser[1]]
        
