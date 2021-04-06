cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS ../components/init1
                         ../components/example_nvs
			 ../components/parse_uuid_data
			 )
			 
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(genX_ble_client2.0)
