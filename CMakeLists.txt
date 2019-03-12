file(GLOB_RECURSE FLEX_PROJECTS_SRCS 
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/ActPack/src/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/Rigid/src/*.c
)

# Add the source to the parent project to create a overall library/DLL
set(FLEX_PROJECTS_SRCS
	${FLEX_PROJECTS_SRCS}
	PARENT_SCOPE
)
