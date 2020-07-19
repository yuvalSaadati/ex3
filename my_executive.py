# name : yuval saadati
# id: 205956634
import sys

from pddlsim.local_simulator import LocalSimulator
from my_agent import Executor
print LocalSimulator().run("simple_football_domain_multi.pddl", "simple_football_problem_multi.pddl", Executor())
#print LocalSimulator().run(str(sys.argv[1]), str(sys.argv[2]), Executor())