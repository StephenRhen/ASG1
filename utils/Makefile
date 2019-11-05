
LDLIBS += -lm

all: sine_table.c

sine_table.c: table_gen
	./table_gen

table_gen: table_gen.c

clean:
	rm table_gen sine_table.c *~
