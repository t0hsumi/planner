NAME = planner

SRC_DIR = srcs/
OBJ_DIR = objs/
DPS_DIR = $(OBJ_DIR)
INC_DIR = include/

CC = g++
BASE_FLAGS = -Wall -MMD -MP -I$(INC_DIR) -I/usr/local/include  -O3 -pthread
CPPFLAGS = $(BASE_FLAGS)

SRCS = $(shell find $(SRC_DIR) -name '*.cpp' | xargs basename -a)
OBJS = $(addprefix $(OBJ_DIR), $(SRCS:.cpp=.o))
DEPS = $(addprefix $(DPS_DIR), $(SRCS:.cpp=.d))

RM = rm
RM_FLAGS = -rf

.PHONY: all
all: $(NAME)

-include $(DEPS)

$(NAME): $(OBJS)
	$(CC) $(CPPFLAGS) $(OBJS) -o $@

$(OBJ_DIR)%.o: $(SRC_DIR)%.cpp
	@if [ ! -d $(OBJ_DIR) ];then mkdir $(OBJ_DIR); fi
	$(CC) $(CPPFLAGS) -o $@ -c $< -MF $(@:%.o=%.d)

.PHONY: clean
clean:
	$(RM) $(RM_FLAGS) $(OBJ_DIR)

.PHONY: fclean
fclean: clean
	$(RM) $(RM_FLAGS) $(NAME)

.PHONY: re
re: fclean all
