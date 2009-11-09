CXXFLAGS += -Wall -Wextra -Iinclude
LDFLAGS += -lcxcore -lcv -lhighgui -lcvaux -lmesasr
OBJS = ./src/cameraImages.o ./src/funcPointing.o ./src/faceDetector.o ./src/regionDetector.o ./src/regionTracker.o ./src/pointProjector.o ./src/coordinateShifter.o ./src/line.o
TARGETS = pointEstimator calibDataCollector

all: $(TARGETS)

$(TARGETS): $(OBJS) ./src/$(TARGETS:=.cpp)
	g++ $(CXXFLAGS) -o ../$@ $(OBJS) $(LDFLAGS)

.PHONY: clean
clean:
	$(RM) *~ */*~ */*.o $(TARGETS)
