# Copyright 2013 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
PROJECT         := IT
DEVICES         := NUCLEO_F401RE
GCC4MBED_DIR    := ../..

#NO_FLOAT_SCANF  := 0
#NO_FLOAT_PRINTF := 0

GCC4MBED_TYPE    = Debug
MBED_LIBS	= rtos


include $(GCC4MBED_DIR)/build/gcc4mbed.mk
dep :		NUCLEO_F401RE-deploy
