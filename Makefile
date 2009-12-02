CXXFLAGS += -Wall -Wextra -Iinclude
LDFLAGS += -lcxcore -lcv -lhighgui -lcvaux -lmesasr
SRCD = ./src/
OBJS = $(SRCD)cameraImages.o $(SRCD)funcPointing.o $(SRCD)faceDetector.o $(SRCD)regionDetector.o $(SRCD)regionTracker.o $(SRCD)pointProjector.o $(SRCD)coordinateShifter.o $(SRCD)line.o $(SRCD)distanceField.o
TARGETS = pointEstimator calibDataCollector

all: $(TARGETS)

$(TARGETS): $(OBJS) $(SRCD)pointEstimator.cpp $(SRCD)calibDataCollector.cpp
	g++ $(CXXFLAGS) -o $@ $(SRCD)$@.cpp $(OBJS) $(LDFLAGS)

.PHONY: clean
clean:
	$(RM) *~ */*~ */*.o $(TARGETS)
