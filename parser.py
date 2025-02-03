#!/usr/bin/env python3

import re
import argparse
import glob
import os

class Pos:
    def __init__(self,x=None,y=None,z=None,dx=None,dy=None,dz=None,abs=False):
        self.pos = {
            "x":x,
            "y":y,
            "z":z,
            "dx":dx,
            "dy":dy,
            "dz":dz,
        };
        self.abs = abs

    def setblock(self,input):
        return "setblock " + self.get_string() + input

    def if_call(self,input):
        return "if block " + self.get_string() + input


    def parse(input):
        ll = input.split();
        result = Pos();
        if ll[0] != 'pos':
            if len(ll) != 3:
                raise Exception(f"Non valid position {input}")
            x = ll[0]
            if ll[0].startswith("~"):
                result.pos['dx'] = int(ll[0][1:])
            else:
                result.pos['x'] = int(ll[0])

            if ll[1].startswith("~"):
                result.pos['dy'] = int(ll[1][1:])
            else:
                result.pos['y'] = int(ll[1])

            if ll[2].startswith("~"):
                result.pos['dz'] = int(ll[2][1:])
            else:
                result.pos['z'] = int(ll[2])

            return result;
        else:
            if ll[1] == 'abs':

                name = ll[2]
                ll = ll[3:]
                parsed = Pos.parse(" ".join(ll))
                parsed.abs = True
                return (name,parsed)
            else:
                name = ll[1]
                ll = ll[2:]
                return (name,Pos.parse(" ".join(ll)))



    def increment(self,dir):
        print(f"incrementing to {dir}")
        if(dir == 'x'):
            return Pos(self.pos['x'] + 1,self.pos['y'],self.pos['z'],self.pos['dx'],self.pos['dy'],self.pos['dz']);
        if(dir == '-x'):
            return Pos(self.pos['x'] - 1,self.pos['y'],self.pos['z'],self.pos['dx'],self.pos['dy'],self.pos['dz']);
        if(dir == 'y'):
            return Pos(self.pos['x'],self.pos['y'] + 1,self.pos['z'],self.pos['dx'],self.pos['dy'],self.pos['dz']);
        if(dir == '-y'):
            return Pos(self.pos['x'],self.pos['y'] - 1,self.pos['z'],self.pos['dx'],self.pos['dy'],self.pos['dz']);
        if(dir == 'z'):
            return Pos(self.pos['x'],self.pos['y'],self.pos['z'] + 1,self.pos['dx'],self.pos['dy'],self.pos['dz']);
        if(dir == '-z'):
            return Pos(self.pos['x'],self.pos['y'],self.pos['z'] - 1,self.pos['dx'],self.pos['dy'],self.pos['dz']);
        if(dir == 'dx'):
            if self.pos['dx'] is None:
                self.pos['dx'] = 0
            return Pos(self.pos['x'],self.pos['y'],self.pos['z'],self.pos['dx'] + 1,self.pos['dy'],self.pos['dz']);
        if(dir == '-dx'):
            if self.pos['dx'] is None:
                self.pos['dx'] = 0
            return Pos(self.pos['x'],self.pos['y'],self.pos['z'],self.pos['dx'] - 1,self.pos['dy'],self.pos['dz']);
        if(dir == 'dy'):
            if self.pos['dy'] is None:
                self.pos['dy'] = 0
            return Pos(self.pos['x'],self.pos['y'],self.pos['z'],self.pos['dx'],self.pos['dy'] + 1,self.pos['dz']);
        if(dir == '-dy'):
            if self.pos['dy'] is None:
                self.pos['dy'] = 0
            return Pos(self.pos['x'],self.pos['y'],self.pos['z'],self.pos['dx'],self.pos['dy'] - 1,self.pos['dz']);
        if(dir == 'dz'):
            if self.pos['dz'] is None:
                self.pos['dz'] = 0
            return Pos(self.pos['x'],self.pos['y'],self.pos['z'],self.pos['dx'],self.pos['dy'],self.pos['dz'] + 1);
        if(dir == '-dz'):
            if self.pos['dz'] is None:
                self.pos['dz'] = 0
            return Pos(self.pos['x'],self.pos['y'],self.pos['z'],self.pos['dx'],self.pos['dy'],self.pos['dz'] - 1);


    def get_string(self):
        output = ""
        if "x" in self.pos and self.pos['x'] is not None:
            if "dx" in self.pos and self.pos['dx'] is not None:
                raise Exception("Sorry, cannot have relative and absolute for same number x")

        if "y" in self.pos and self.pos['y'] is not None:
            if "dy" in self.pos and self.pos['dy'] is not None:
                raise Exception("Sorry, cannot have relative and absolute for same number y")

        if "z" in self.pos and self.pos['z'] is not None:
            if "dz" in self.pos and self.pos['dz'] is not None:
                raise Exception("Sorry, cannot have relative and absolute for same number z")

        if "x" in self.pos and self.pos['x'] is not None:
            output += str(self.pos['x'])+ " ";
        elif "dx" in self.pos:
            if self.pos['dx'] is None:
                output += "~0 "
            else:
                output += "~"+str(self.pos['dx']) + ' '

        if "y" in self.pos and self.pos['y'] is not None:
            output += str(self.pos['y'])+ " ";
        elif "dy" in self.pos:
            if self.pos['dy'] is None:
                output += "~0 "
            else:
                output += "~"+str(self.pos['dy']) + ' '
        if "z" in self.pos and self.pos['z'] is not None:
            output += str(self.pos['z']) + " ";
        elif "dz" in self.pos:
            if self.pos['dz'] is None:
                output += "~0 "
            else:
                output += "~"+str(self.pos['dz']) + ' '

        caps = "" if self.abs == False else "|"
        return caps+output+caps
            
class Block:
    def get(pos: Pos, command, kind, conditional, state, direction):

        print(f"block looking at pos {pos}")
        return {
            "pos": pos.get_string(),
            'command':command,
            'kind':kind,
            'conditional':conditional,
            'state':state,
            'direction':direction
        }

class ScoreBoard:
    def parse(program, input):
        input = input.rstrip(';')
        items = input.split();
        if(items[0] != "scoreboard"):
            raise Exception(f"not a scoreboard {input}")
        items = items[1:]
        for item in items:
            if item in program.scoreboards:
                raise Exception(f"Scoreboard {item} already exists in program")
            else:
                program.scoreboards[item] = ScoreBoard(program,item)


    def __init__(self,program, name):
        self.program = program;
        self.name = name

    def get_name(self):
        return self.program.name + "_" + self.name

class Score:
    def __init__(self,parent: ScoreBoard, name,collection=None):
        self.parent = parent
        self.program = parent.program;
        self.name = name;
        self.collection = collection;

    def parse(program, input, collection=None):
        input = input.rstrip(';')
        items = input.split();
        if(items[0] != "score"):
            raise Exception(f"not a score {input}")

        items = items[1:]
        scoreboard = items[0]
        if scoreboard not in program.scoreboards:
                raise Exception(f"Scoreboard {scoreboard} does not exist in program")
        scoreboard = program.scoreboards[scoreboard]
        return Score(scoreboard,items[1],collection)


    def get_name(self):
        return self.collection + "_" + self.name

    def operation(self,operator,other_score):
        return (
            f"scoreboard players operation {self.get_name()} {self.parent.get_name()}"
            f"{operator}{other_score.get_name()} {other_score.parent.get_name()}"
        )

    def set(self,input):
        return f"scoreboard players set {self.get_name()} {self.parent.get_name()} {input}"

    def store(self,input,arg="result",start="run "):
        return (
            f"store {arg} score {self.get_name()} {self.parent.get_name()} "
            f"{start}{input}"
        )

class Dataref:
    def __init__(self, name, kind, ident):
        self.kind = kind;
        self.name = name
        if(kind == 'block'):
            self.pos = ident;
        if(kind == 'entity'):
            self.ent = ident
        if(kind == 'storage'):
            self.target = ident

    def get_ident(self):
        if self.kind != 'block':
            raise Exception("data reference type {self.kind} not supported, make a pr")
        return self.pos.get_string()

    def get(self,input):
       return f"data get {self.kind} {self.get_ident()}{input}"

    def parse(input):
        input = input.rstrip(';')
        items = input.split();
        if(items[0] != 'dataref'):
            raise Exception(f"Not a dataref {input}")
        name = items[1];
        kind = items[2];
        if(kind != 'block'):
            raise Exception(f"Dataref kind {kind} not implemented yet")
        pos = Pos.parse(" ".join([items[3],items[4],items[5]]))
        result = Dataref(name, kind, pos)
        return result



class Program:
    def __init__(self,input_text):
        self.scoreboards = [];
        self.if_order = ['dx','dz','-dx','dz']
        self.if_dir = ['east','south','west','south']
        self.block_count = 0
        self.depth = 0
        self.last = [Pos(dx=0,dy=0,dz=0)];
        self.blocks = [];
        self.scoreboards = {};
        self.collections = {};
        self.variables = {};
        self.original_input = self.convert_case_to_if(input_text)
        self.name = None
        print(self.last)
        self.parse()
        self.generate()

    def parse_collection(self,name,input_lines):
        # expects the body lines
        name = '#' + name;
        print(f"parsing collection {name}")
        if name in self.collections:
            raise Exception("collection {name} already exists for program")
        result = {
            "scores": {},
            "poss": {},
            "refs": {}
        }
        for line in input_lines:
            line = line.strip();
            line = line.rstrip(';')
            line = line.rstrip('}')
            if len(line) == 0:
                continue
            words = line.split()
            print(f"words {words}")
            if words[0] == 'score':
                score = Score.parse(self,line,name)
                if score.name in result['scores']:
                    raise Exception(f"Name {score.name} from {line} already used")
                if score.name in result['poss']:
                    raise Exception(f"Name {score.name} from {line} already used")
                if score.name in result['refs']:
                    raise Exception(f"Name {score.name} from {line} already used")
                result['scores'][score.name] = score;
            elif words[0] == 'dataref':
                dataref = Dataref.parse(line)
                if dataref.name in result['scores']:
                    raise Exception(f"Name {dataref.name} from {line} already used in scores")
                if dataref.name in result['poss']:
                    raise Exception(f"Name {dataref.name} from {line} already used in positions")
                if dataref.name in result['refs']:
                    raise Exception(f"Name {dataref.name} from {line} already used in refs")
                result['refs'][dataref.name] = dataref;
            elif words[0] == 'pos':
                pos = Pos.parse(line)
                name_p = pos[0];
                data = pos[1]
                if name_p in result['scores']:
                    raise Exception(f"Name {name_p} from {line} already used in scores")
                if name_p in result['poss']:
                    raise Exception(f"Name {name_p} from {line} already used in positions")
                if name_p in result['refs']:
                    raise Exception(f"Name {name_p} from {line} already used in refs")
                result['poss'][name_p] = data;
            else:
                raise Exception(f"{word[0]} Not implemented for collection parsing")

        print(f"collection yeilded {result}")
        self.collections[name] = result



    def convert_case_to_if(self,input_code):
        lines = input_code.splitlines()
        output_lines = []
        i = 0

        while i < len(lines):
            line = lines[i].strip()

            if line.startswith('case('):
                # Extract the variable inside case()
                case_var = line[line.find('(')+1:line.find(')')].strip()
                i += 1

                # Initialize bracket counter
                bracket_count = 1  # The opening '{' after 'case'

                # Process each case entry
                case_body_lines = []
                while i < len(lines) and bracket_count > 0:
                    entry_line = lines[i]
                    bracket_count += entry_line.count('{')
                    bracket_count -= entry_line.count('}')
                    case_body_lines.append(entry_line)
                    i += 1

                # Process the collected case body
                j = 0
                while j < len(case_body_lines):
                    entry_line = case_body_lines[j].strip()

                    if '=>' in entry_line:
                        # Extract conditions and prepare for body
                        conditions_part, _ = entry_line.split('=>', 1)
                        conditions = [cond.strip().strip("'") for cond in conditions_part.split(',') if cond.strip()]

                        # Collect the body block
                        body_lines = []
                        if '{' in entry_line:
                            j += 1
                        else:
                            j += 1
                            continue

                        # Read until the closing '}'
                        inner_bracket_count = 1  # Opening '{' for this case
                        while j < len(case_body_lines) and inner_bracket_count > 0:
                            current_line = case_body_lines[j]
                            inner_bracket_count += current_line.count('{')
                            inner_bracket_count -= current_line.count('}')

                            if inner_bracket_count > 0:
                                body_lines.append(current_line.strip())
                            j += 1

                        # Create if statements for each condition
                        body = '\n    '.join([line for line in body_lines if line])
                        for condition in conditions:
                            output_lines.append(f"if({case_var} == {condition}){{\n    {body}\n}}")
                    else:
                        j += 1
            else:
                output_lines.append(line)
                i += 1

        return '\n'.join(output_lines)




    def parse_name(self):
        # Look for the @name = <name>; pattern
        match = re.search(r'@name\s?=\s?(\w+);', self.original_input)
        if match:
            self.name = match.group(1)
        else:
            raise Exception("Program name not found in input")

    def parse_initial(self):
        # Look for the @initial{...} block
        match = re.search(r'@initial{(.*)};\n', self.original_input, re.DOTALL)
        if match:
            initial_block = match.group(1)
            # Split by lines and parse each line in the @initial block
            initial_lines = initial_block.strip().splitlines()
            count = 1;
            new_block = []
            for line in initial_lines:
                if "{" in line:
                    count += 1;
                if "}" in line:
                    count -= 1;
                if count == 0:
                    break;
                new_block.append(line)
            initial_block = "\n".join(new_block);
            initial_lines = new_block
            print(f"initial lines {initial_lines}")
            for line in initial_lines:
                line = line.strip()
                if line.startswith('scoreboard'):
                    ScoreBoard.parse(self, line)
                elif line.startswith('collection'):
                    collection_name = re.search(r'#(\w+)', line).group(1)
                    # Extract the collection block (i.e., contents inside {...})
                    collection_block = re.search(r'\{(.*)\}', initial_block, re.DOTALL).group(1)
                    self.parse_collection(collection_name, collection_block.splitlines())
                else:
                    print(f"Unknown directive in @initial: {line}")
        else:
            raise Exception("Initial block not found in input")
        # generate blocks
        for key,value in self.scoreboards.items():
            self.add_block(
                f"scoreboard objectives add {value.get_name()} dummy",
                "impulse",
                "false",
                "always"
            );

    def resolve_name(self,input):
        temp = input.split('.')
        if (len(temp) == 0):
            raise Exception("Not a name to resolve {input}")
        temp[0] = temp[0].strip(";")
        if temp[0].isnumeric():
            return "NUMBER", int(temp[0]), temp[1:]

        if temp[0] in self.collections:
            col = self.collections[temp[0]]
            temp = temp[1:]
            temp[0] = temp[0].strip(';')
            print(f"looking at temp for scores {temp}")
            print(f'col scores is {col["scores"]}')
            if temp[0].split(" %= ")[0] in col['scores']:
                print(f'looking at score {temp[0]} as {temp[0].split(" %= ")}')
                score = col['scores'][temp[0].split()[0]]
                return "SCORE", score, temp[0].split()[1:]+temp[1:]
            if temp[0].split(" /= ")[0] in col['scores']:
                print(f'looking at score {temp[0]} as {temp[0].split(" / ")}')
                score = col['scores'][temp[0].split()[0]]
                return "SCORE", score, temp[0].split()[1:]+temp[1:]
            if temp[0].split(" -= ")[0] in col['scores']:
                print(f'looking at score {temp[0]} as {temp[0].split(" -= ")}')
                score = col['scores'][temp[0].split()[0]]
                return "SCORE", score, temp[0].split()[1:]+temp[1:]
            if temp[0].split(" += ")[0] in col['scores']:
                print(f'looking at score {temp[0]} as {temp[0].split(" += ")}')
                score = col['scores'][temp[0].split()[0]]
                return "SCORE", score, temp[0].split()[1:]+temp[1:]
            if temp[0].split(" *= ")[0] in col['scores']:
                print(f'looking at score {temp[0]} as {temp[0].split(" *= ")}')
                score = col['scores'][temp[0].split()[0]]
                return "SCORE", score, temp[0].split()[1:]+temp[1:]
            if temp[0].split(" = ")[0] in col['scores']:
                print(f'looking at score {temp[0]} as {temp[0].split(" = ")}')
                score = col['scores'][temp[0].split()[0]]
                return "SCORE", score, temp[0].split()[1:]+temp[1:]
            if temp[0].split(" == ")[0] in col['scores']:
                score = col['scores'][temp[0].split()[0]]
                return "SCORE", score, temp[0].split()[1:]+temp[1:]
            if temp[0] in col['poss']:
                print(f"pos temp[0] is {temp[0]}")
                pos = col['poss'][temp[0]]
                if(temp[1].startswith(".")):
                    temp[1] = temp[1][1:]
                print(f"pos temp1 is {temp[1]}")
                if( len(temp) > 0 and temp[1].startswith("setblock(") ):
                    s = temp[1]
                    s = s[s.find('(')+1:s.find(')')]
                    return "SETBLOCK", pos.setblock(s), temp[2:]
                if( len(temp) > 0 and temp[1].startswith("if(") ):
                    s = temp[1]
                    s = s[s.find('(')+1:s.find(')')]
                    return "BLOCK_IF", pos.if_call(s), temp[2:]
                return "POS", pos, temp[1:]
            if temp[0] in col['refs']:
                refs = col['refs'][temp[0]]
                print(f"first {temp[1]}")
                if(temp[1].startswith(".")):
                    temp[1] = temp[1][1:]
                if( len(temp) > 0 and temp[1].startswith("get(") ):
                    s = temp[1]
                    s = s[s.find('(')+1:s.find(')')]
                    return "DATAREF_GET", refs.get(s), temp[2:]
                return "DATAREF", refs, temp[1:]
        else:
            print(f"{temp[0]} not in {self.collections}")


        raise Exception(f"not implemented to find {input}")

    def handle_expression(self,input):
        resolved = self.resolve_name(input);
        print(f"resolved {resolved}")
        if(resolved[0] == 'SETBLOCK'):
            return resolved[1]
        if(resolved[0] == 'BLOCK_IF'):
            return resolved[1]


        if(resolved[0] == 'DATAREF_GET'):
                raise Exception("Not implemented {input} make a pr")
        if(resolved[0] == 'SCORE'):
            score = resolved[1]

            if resolved[2][0] == '%=':
                input = resolved[2][1:]
                input = ".".join(input)
                print("starts with nothing")
                print(f"input {input}")
                get = self.resolve_name(input)
                print(f"get is {get}")
                if get[0] == "SCORE":
                    return score.operation(" %= ",get[1])
            if resolved[2][0] == '/=':
                input = resolved[2][1:]
                input = ".".join(input)
                print("starts with nothing")
                print(f"input {input}")
                get = self.resolve_name(input)
                print(f"get is {get}")
                if get[0] == "SCORE":
                    return score.operation(" /= ",get[1])
            if resolved[2][0] == '-=':
                input = resolved[2][1:]
                input = ".".join(input)
                print("starts with nothing")
                print(f"input {input}")
                get = self.resolve_name(input)
                print(f"get is {get}")
                if get[0] == "SCORE":
                    return score.operation(" -= ",get[1])
            if resolved[2][0] == '+=':
                input = resolved[2][1:]
                input = ".".join(input)
                print("starts with nothing")
                print(f"input {input}")
                get = self.resolve_name(input)
                print(f"get is {get}")
                if get[0] == "SCORE":
                    return score.operation(" += ",get[1])
                else:
                    raise Exception(f"Not implemented {input} make a pr")
            if resolved[2][0] == '*=':
                input = resolved[2][1:]
                input = ".".join(input)
                print("starts with nothing")
                print(f"input {input}")
                get = self.resolve_name(input)
                print(f"get is {get}")
                if get[0] == "SCORE":
                    return score.operation(" *= ",get[1])
                else:
                    raise Exception(f"Not implemented {input} make a pr")

            elif resolved[2][0] == '=':
                input = resolved[2][1:]
                input = ".".join(input)
                print("starts with nothing")
                print(f"input {input}")
                get = self.resolve_name(input)
                print(f"get is {get}")
                if get[0] == "DATAREF_GET":
                    return score.store(get[1])
                elif get[0] == "NUMBER":
                    return score.set(get[1])
                elif get[0] == "SCORE":
                    return score.operation(" = ",get[1])
                elif get[0] == "BLOCK_IF":
                    return score.store(get[1],arg="success",start="")
                else:
                    raise Exception(f"Not implemented {input} make a pr")
            else:
                input = ".".join(resolved[2]).strip();
            if(input.startswith(".")):
                print("starts with .")
                input = input[1:]
            if (input.startswith("==")):
                print("starts with == ")
                input = input.split("==")[1]
                if input.startswith("."):
                    input = input[1:]
                return f"score {score.get_name()} {score.parent.get_name()} matches {input}"

        raise Exception(f"Not implemented {input} make a pr")


    def parse_body(self,body):
        lines = body.splitlines()
        i = 0
        while i < len(lines):
            if lines[i].startswith("if"):
                s = lines[i]
                cond = s[s.find('(')+1:s.find(')')]
                cmd = self.handle_expression(cond)
                cond = cmd
                self.add_if(cmd,"seed");
                i += 1
                while i < len(lines) and '}' not in lines[i]:
                    cmd = self.handle_expression(lines[i].strip())
                    self.add_if(cond,cmd);
                    i += 1
                if '}' in lines[i]:
                    i += 1;
                self.last = [self.last[0]]

            else:
                cmd = self.handle_expression(lines[i].strip())
                self.add_block(cmd,"chain","false","always")
                i += 1

        
    def parse_always(self):
        pattern = re.compile(r"@always\[(?P<FLAGS>.*?)\]\((?P<ALWAYS_EXPR>.*?)\)\{(?P<ALWAYS_BODY>.*?)\};", re.DOTALL)

        match = pattern.search(self.original_input)

        if match:
            flags = match.group("FLAGS").strip()
            always_expr = match.group("ALWAYS_EXPR").strip()
            always_body = match.group("ALWAYS_BODY").strip()

            print(f"expre is {always_expr}")
            always_cmnds = always_expr.splitlines();
            for expr in always_cmnds:
                expr = expr.strip().strip(";")
                expr_command = self.handle_expression(expr)
                self.add_block("execute "+expr_command,'repeat','false','always')

            self.parse_body(always_body)

            print("FLAGS:", flags)
            print("ALWAYS_EXPR:", always_expr)
            print("ALWAYS_BODY:", always_body)
        else:
            print("No match found.")

    def parse_end(self,increment=True):
        pattern = re.compile(r"@end\{(?P<END_BODY>.*?)\};", re.DOTALL)
        match = pattern.search(self.original_input)

        if match:
            end_body = match.group("END_BODY").strip()
            print(f"end body {end_body}")
            lines = end_body.splitlines();
            result = []
            for line in lines:
                expr_command = self.handle_expression(line)
                if increment:
                    self.add_block(expr_command,'impulse','false','always',increment=increment);
                else:
                    result.append(self.add_block(expr_command,'impulse','false','always',increment=increment))
            return result



    def parse(self):
        self.parse_name(); # parse @name = <name>
        self.parse_initial();
        self.parse_always();
        self.parse_end()


    def insert_blocks(self,block):
        if len(self.blocks) == 0:
            self.blocks.append([])

        if(len(self.blocks[-1]) >= 100):
            print(f"tyring to append {block}")
            old_depth = self.depth
            old_block_count = self.block_count
            old_last = self.last[0]
            temp = self.parse_end(increment=False)
            for new_block in temp:
                 self.blocks[-1].append(new_block)
            self.depth = old_depth
            self.block_count = old_block_count
            self.last[0] = old_last
            self.blocks.append([])

        self.blocks[-1].append(block)

    def add_if(self,cond,command):
        command = "execute if "+cond+ " run " + command
        self.add_block(command,'chain','false','always')


    ## TODO @(dleiferives,b1ed2a47-bbaf-457a-9854-00ce7dca7c3f): make it go in a
    ## cube ~#
    def add_block(self,command,kind,conditional,state,increment=True):
        old_depth = self.depth
        old_block_count = self.block_count
        self.depth = self.depth % len(self.if_order);
        if self.depth == 0 and self.block_count == 7:
            self.depth += 1;
            self.block_count = 0;
        elif self.depth == 1 and self.block_count == 1:
            self.depth += 1;
            self.block_count = 0;
        elif self.depth == 2 and self.block_count == 7:
            self.depth += 1
            self.block_count = 0;
        elif self.depth == 3 and self.block_count == 1:
            self.depth += 1;
            self.block_count = 0;
        depth = self.depth % len(self.if_order);
        if self.block_count == 0 and old_block_count != self.block_count:
            current_pos = self.last[0].increment(self.if_order[depth-1])
        else:
            current_pos = self.last[0].increment(self.if_order[depth])

        print(f"{self.last[0]}")
        print(f"generating block at {current_pos}")
        new = Block.get(current_pos,command,kind,conditional,state,self.if_dir[depth])
        if increment:
            self.last[0] = current_pos
            self.block_count += 1
            self.insert_blocks(new)
        else:
            self.last[0] = current_pos
            self.block_count += 1
            return new


    def generate(self):
        def flip_signs(content):
            # Flip signs for both regular numbers and ~ prefixed numbers
            flipped = re.sub(r'(~?)([-+]?\d+)', lambda m: m.group(1) + str(-int(m.group(2))), content)
            return flipped

        def add_values(a, c):
            a_parts = a.strip('|').split()
            c_parts = c.split()

            result = []
            for a_val, c_val in zip(a_parts, c_parts):
                a_num = int(a_val.replace('~', ''))
                c_num = int(c_val.replace('~', ''))

                # Preserve '~' if present in original value
                tilde = '~' if '~' in a_val or '~' in c_val else ''
                result.append(f"{tilde}{a_num + c_num}")

            temp = ' '.join(result) +' '
            return temp

        def replace_with_flipped_signs(input_text, B):
            C = flip_signs(B)

            def replacer(match):
                A = match.group(0)
                return add_values(A, C)

            return re.sub(r'\|[^|]*\|', replacer, input_text)

        command = r'/summon falling_block ~ ~1 ~ {BlockState:{Name:"activator_rail"},Time:1,Passengers:['
        prelude = "\n"+r'{id:command_block_minecart,Command:"setblock ~-2 ~-1 ~ minecraft:command_block{Command:\"setblock ~2 ~1 ~ lava[level=15]\"}"},'
        command += prelude
        # prelude = "\n"+r'{id:command_block_minecart,Command:"setblock ~-4 ~-1 ~ minecraft:command_block{Command:\"fill ~4 ~1 ~ ~'+str(4+len(commands))+r' ~1 ~ air\"}"},'
        # command += prelude
        command_start = command;
        end =  r']}'

        result = []
        for blocks in self.blocks:
            command = command_start;
            for block in blocks:
                c = block['command'];
                c = c.replace('\"','\\\\\\"')
                pos = block['pos']
                c = replace_with_flipped_signs(c,pos)
                kind = block['kind']
                if kind ==  'impulse':
                    kind = 'command_block'
                elif kind == 'chain':
                    kind= 'chain_command_block'
                else:
                    kind = 'repeating_command_block'

                prelude = "\n"+r'{id:command_block_minecart,Command:"setblock '+pos+r'minecraft:'+kind+r'[facing='+block['direction']+r',conditional='+block['conditional']+r']{auto:1b,Command:\"'+str(c)+r'\"}"},'
                command += prelude
            command = command + end
            result.append(command)


        self.command = "\n\n".join(result);
        # print(command+end)
        return "\n\n".join(result)





demo = """
@name = H1P2;
@initial{
    scoreboard INPUT;
    collection #L1{
        score INPUT score;
        dataref ref block 996 56 8;
        pos line1 997 56 9;
        pos line2 995 56 9;
    };
};

@always[REDSTONE]( #L1.score = #L1.ref.get(Page)){
    case(#L1.score){
        '5','0' => {
            #L1.line1.setblock(minecraft:air);
            #L1.line2.setblock(minecraft:air);
        };
        '1' => {
            #L1.line1.setblock(minecraft:air);
            #L1.line2.setblock(minecraft:redstone_block);
        };
        '2' => {
            #L1.line1.setblock(minecraft:redstone_block);
            #L1.line2.setblock(minecraft:air);
        };
        '3' => {
            #L1.line1.setblock(minecraft:redstone_block);
            #L1.line2.setblock(minecraft:redstone_block);
        };

    };
};
"""

def process_file(input_file):
    with open(input_file, 'r') as file:
        input_text = file.read()

    # Generate the output using the Program class
    program = Program(input_text)

    output_file = os.path.splitext(input_file)[0] + '.mcmb'
    with open(output_file, 'w') as out_file:
        out_file.write(program.generate())

    print(f"Generated output for {input_file} -> {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Process .red files to generate .mcmb output files.')
    parser.add_argument('path', help='Path to .red file or directory containing .red files')
    args = parser.parse_args()

    if os.path.isdir(args.path):
        files = glob.glob(os.path.join(args.path, '*.red'))
    elif os.path.isfile(args.path) and args.path.endswith('.red'):
        files = [args.path]
    else:
        print("Invalid path. Please provide a .red file or a directory containing .red files.")
        return

    for file in files:
        process_file(file)

if __name__ == '__main__':
    main()
