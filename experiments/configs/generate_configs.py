maps = ("den520d", "empty256", "maze", "room", "random", "warehouse", "Berlin")
generators = ("CDT", "kPRM")
n_vs = (100, 700, 5000)
for mp in maps:
    for gen in generators:
        for n in n_vs:
            name = mp + '_' + str(n) + '_' + gen
            with open(name+".yaml","w") as out:
                out.write("Generator:\n")
                out.write("  type: "+gen+"\n")
                if gen == "kPRM":
                    out.write("  number_of_vertices: 0\n")
                    out.write("  number_of_neighbors: 15\n")
                out.write("  \n")
                out.write("Input:\n")
                out.write("  type: Text\n")
                out.write("  file: ../problem_instances/spatial/" +mp+ ".txt\n")
                out.write("\n")
                out.write("Roadmaps: ../experiments/roadmaps/" +name+ "/\n")
                out.write("Iterations: 25\n")
                out.write("number_of_agents: " + str(n) + "\n")
                out.write("\n")
                remain = [
                    "Planners:\n",
                    "  PSIPP:\n",
                    "    type: PSIPP\n",
                    "    time_limit: 30.0\n",
                    "  CCBS:\n",
                    "    type: CCBS\n",
                    "    time_limit: 30.0\n",
                    "PairsToRun:\n",
                    "  - Problem: Unannotated\n",
                    "    Planner: CCBS\n",
                    "  - Problem: Unannotated\n",
                    "    Planner: PSIPP\n",
                    "  - Problem: Annotated\n",
                    "    Planner: PSIPP"]
                out.writelines(remain)
