'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-04 19:36:30
LastEditTime: 2022-09-04 19:36:36
Description: 
'''
import os
import time

ret = 99
resume_count = 0
while ret and resume_count < 10:
    resume_count += 1
    ret = os.system('python naive_rendering.py')
    time.sleep(1)
    ret = ret // 256
    print(f'resume: {resume_count}')
    
print('Finished')