#compiler
OMPL_DIR = /usr/local
INC_CLASSES = /home/avishai/Downloads/omplapp/ompl/Workspace/cplanner/checker_classes/
INC_PLANNERS = /home/avishai/Downloads/omplapp/ompl/Workspace/cplanner/planners/
INC = /home/avishai/Downloads/omplapp/ompl/Workspace/cplanner/

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lXmu -lXi -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${INC_PLANNERS} -I${PQP_DIR}/include $(GL_INCPATH)
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -L${PQP_DIR}/lib -lPQP -lm $(GL_LIBS) 
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system
CPPSD = ${INC_CLASSES}Rod_ODE_class.cpp ${INC_CLASSES}robots_class.cpp
CPPP = ${INC_CLASSES}Rod_ODE_class.cpp ${INC_CLASSES}robots_class.cpp ${INC_PLANNERS}PCP.cpp StateValidityCheckerClass.cpp 
CPPS = ${INC_CLASSES}Rod_ODE_class.cpp ${INC_CLASSES}robots_class.cpp ${INC_PLANNERS}myRRTConnect.cpp StateValidityCheckerClass.cpp 
CPPT = ${INC_CLASSES}Rod_ODE_class.cpp
CPPPr = ${INC_CLASSES}Rod_ODE_class.cpp ${INC_CLASSES}robots_class.cpp ${INC_PLANNERS}decoupled_rod.cpp ${INC_PLANNERS}myRRTConnect.cpp StateValidityCheckerClass.cpp
CPPPQP = ${INC_CLASSES}collisionDetection.cpp ${INC_CLASSES}model.cpp

all:
	$(CXX) main.cpp $(CPPSD) $(CPPPQP) -o gen $(CXXFLAGS) $(LDFLAGS) -std=c++11 #-Wall		





