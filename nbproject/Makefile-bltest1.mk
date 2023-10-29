#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-bltest1.mk)" "nbproject/Makefile-local-bltest1.mk"
include nbproject/Makefile-local-bltest1.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=bltest1
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/bootloader_pic.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/bootloader_pic.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=bltest1.s

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/bltest1.o
POSSIBLE_DEPFILES=${OBJECTDIR}/bltest1.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/bltest1.o

# Source Files
SOURCEFILES=bltest1.s



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-bltest1.mk ${DISTDIR}/bootloader_pic.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=PIC16F1574
FINAL_IMAGE_NAME_MINUS_EXTENSION=${DISTDIR}/bootloader_pic.${IMAGE_TYPE}
# ------------------------------------------------------------------------------------
# Rules for buildStep: pic-as-assembler
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/bltest1.o: bltest1.s  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/bltest1.o 
	${MP_AS} -mcpu=PIC16F1574 -c \
	-o ${OBJECTDIR}/bltest1.o \
	bltest1.s \
	 -D__DEBUG=1   -mdfp="${DFP_DIR}/xc8"  -msummary=+mem,+psect,+class,-hex,-file,-sha1,-sha256,-xml,-xmlfull -v -fmax-errors=20 -mwarn=0 -xassembler-with-cpp -Wa,-a
	
else
${OBJECTDIR}/bltest1.o: bltest1.s  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/bltest1.o 
	${MP_AS} -mcpu=PIC16F1574 -c \
	-o ${OBJECTDIR}/bltest1.o \
	bltest1.s \
	  -mdfp="${DFP_DIR}/xc8"  -msummary=+mem,+psect,+class,-hex,-file,-sha1,-sha256,-xml,-xmlfull -v -fmax-errors=20 -mwarn=0 -xassembler-with-cpp -Wa,-a
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: pic-as-linker
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/bootloader_pic.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    bootload.lkr
	@${MKDIR} ${DISTDIR} 
	${MP_LD} -mcpu=PIC16F1574 ${OBJECTFILES_QUOTED_IF_SPACED} \
	-o ${DISTDIR}/bootloader_pic.${IMAGE_TYPE}.${OUTPUT_SUFFIX} \
	 -D__DEBUG=1   -mdfp="${DFP_DIR}/xc8"  -msummary=+mem,+psect,+class,-hex,-file,-sha1,-sha256,-xml,-xmlfull -Wl,-PResVect=0x00,-PIntVect=0x004,-PMain=0x008 -mcallgraph=std -Wl,-Map=${FINAL_IMAGE_NAME_MINUS_EXTENSION}.map -mno-download-hex -Wa,-a
else
${DISTDIR}/bootloader_pic.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   bootload.lkr
	@${MKDIR} ${DISTDIR} 
	${MP_LD} -mcpu=PIC16F1574 ${OBJECTFILES_QUOTED_IF_SPACED} \
	-o ${DISTDIR}/bootloader_pic.${IMAGE_TYPE}.${OUTPUT_SUFFIX} \
	  -mdfp="${DFP_DIR}/xc8"  -msummary=+mem,+psect,+class,-hex,-file,-sha1,-sha256,-xml,-xmlfull -Wl,-PResVect=0x00,-PIntVect=0x004,-PMain=0x008 -mcallgraph=std -Wl,-Map=${FINAL_IMAGE_NAME_MINUS_EXTENSION}.map -mno-download-hex -Wa,-a
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
