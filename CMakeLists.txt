file(GLOB_RECURSE FLEX_PROJECTS_SRCS 
	ActPack/*.c
	Rigid/*.c
	src/*.c
)

# Add the source to the parent project to create a overall library/DLL
set(FLEX_PROJECTS_SRCS
	${FLEX_PROJECTS_SRCS}
	PARENT_SCOPE
)
