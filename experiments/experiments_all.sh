maps=(den520d empty256 room maze random warehouse Berlin)
settings=(_100_kPRM
	  _100_CDT
	  _700_kPRM
	  _700_CDT
	  _5000_kPRM
	  _5000_CDT
	 )

mkdir -p ../experiments/roadmaps/ ../experiments/results/
for setting in ${settings[@]};
do
    for map in ${maps[@]};
    do
	experiment=$map$setting
	if [ ! -s ../experiments/results/$experiment.txt ]; then
	    echo $experiment
	
	    mkdir -p ../experiments/roadmaps/$experiment
	    config=../experiments/configs/$experiment.yaml
	    ../build/make_roadmaps $config
	    result=../experiments/results/$experiment.txt
	    ../build/planning_incrementally $config > $result
	fi
    done
done

