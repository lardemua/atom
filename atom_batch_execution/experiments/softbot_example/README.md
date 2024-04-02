# Softbot batch execution example

In this script we will carry out the batch execution of two experiments using a softbot train dataset.

We have two concepts which are important to understand what follows: an **experiment** is an execution of one or more atom scripts in sequence, with specific parameters given as command line arguments to those scripts. A experiment may have different **runs**, which are repetitions of an experiment with the same parameters expect for those related to randomness, i.e. the random seed of the experiment. These runs are then averaged out using the process_results script, in order to provide statistically significant results.

To run, first go to the the experiments/example

    roscd atom_batch_execution/experiments/example/

then do:

    rosrun atom_batch_execution batch_execution -tf template.yml.j2 -df data.yml -of results -v -ow

This will create a _results_ folder and put all runs inside. Finally, if you want to average the numerical cells in collected csv files you can use:

    rosrun atom_batch_execution process_results -rf results/ -of results_processed -ow -v

This command will average out all runs from the same experiments. The created folder _results_processed_, will contain experiments (instead of runs of experiments).
