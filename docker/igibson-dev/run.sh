#!/bin/bash
# This job should be run on the SC headnode.
# Usage: sbatch --export=IG_IGIBSON_PATH=path,IG_OUTPUT_PATH=path,IG_ENTRYPOINT_MODULE=path run.sh
#SBATCH --partition=svl --qos=normal
#SBATCH --ntasks=32
#SBATCH --nodes=1-5
#SBATCH --time=48:00:00
#SBATCH --cpus-per-task=2
#SBATCH --ntasks-per-gpu=2
#SBATCH --mem-per-cpu=8G
#SBATCH --job-name="vision-dataset-generation"
#SBATCH --output=logs/%x_%A_%2t.out
#SBATCH --error=logs/%x_%A_%2t.err
#SBATCH --gpu-bind=single:2

######################
# Begin work section #
######################
echo "SLURM_JOBID="$SLURM_JOBID
echo "SLURM_LOCALID="$SLURM_LOCALID
echo "SLURM_JOB_NODELIST"=$SLURM_JOB_NODELIST
echo "SLURM_NNODES"=$SLURM_NNODES
echo "SLURMTMPDIR="$SLURMTMPDIR
echo "working directory = "$SLURM_SUBMIT_DIR

# Then, create a container.
enroot create -n igibson /cvgl/group/igibson-docker/igibson-dev.sqsh && {
  # Run the container, mounting iGibson at the right spot
  enroot start -r -w \
    -m ${IG_IGIBSON_PATH}:/igibson \
    -m ${IG_OUTPUT_PATH}:/out \
    -e SLURM_LOCALID=${SLURM_LOCALID} \
    -e IG_ENTRYPOINT_MODULE=${IG_ENTRYPOINT_MODULE} \
    igibson;
  # Remove the container.
  enroot remove -f igibson;
}