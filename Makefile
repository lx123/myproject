all: fmu_default

ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))

#make同时执行命令数目
j ?= 4

PX4_CMAKE_GENERATOR ?= "Unix Makefiles"
PX4_MAKE = make
PX4_MAKE_ARGS = -j$(j) --no-print-directory

#描述怎样建立cmake配置
define cmake-build
+@if [ ! -e $(PWD)/build_$@/CMakeCache.txt ]; then mkdir -p $(PWD)/build_$@ && cd $(PWD)/build_$@ && cmake .. -G$(PX4_CMAKE_GENERATOR) -DCONFIG=$(1); fi
+$(PX4_MAKE) -C $(PWD)/build_$@ $(PX4_MAKE_ARGS) $(ARGS)
endef

# create empty targets to avoid msgs for targets passed to cmake
define cmake-targ
$(1):
	@#
.PHONY: $(1)
endef

#ADD CONFIGS HERE  添加配置伪目标

fmu_default:
	$(call cmake-build,nuttx_fmu_default)


clean:
	@rm -rf build_*/
.PHONY: clean

