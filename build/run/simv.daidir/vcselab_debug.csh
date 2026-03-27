#!/bin/csh -f

cd /home/stanley/stanley/TPU_FP32_PIPELINE

#This ENV is used to avoid overriding current script in next vcselab run 
setenv SNPS_VCSELAB_SCRIPT_NO_OVERRIDE  1

/export/SoftWare/Synopsys/vcs/T-2022.06/linux64/bin/vcselab $* \
    -o \
    /home/stanley/stanley/TPU_FP32_PIPELINE/build/run/simv \
    -nobanner \

cd -

