# Perl script to easily launch AT91 debug sessions.

use File::Basename;

# List of supported boards
my @boards = ("atsam3s-vb",
              "atsam3s-ek"
             );

# Check that an argument has been provided
if (!@ARGV[0]) {

   print("Usage: " . basename($0) . " <elf-file>\n");
   exit(1);
}

# Parse file name
my $file = @ARGV[0];
my $script = "";
my $gdb = dirname($0).'/';
my $scripts  = {
    'atsam3s-vb' => {
        'sram'  => {'post' => $gdb.'atsam3s-vb-sram.gdb'},
        },
    'atsam3s-ek' => {
        'sram'  => {'post' => $gdb.'atsam3s-ek-sram.gdb'},
        }
};

# Check #2: this must be an elf file
if ($file !~ m/.*.elf/i) {

   print(".elf file expected.\n");
   exit(2);
}

# Check #1: 'sdram' or 'ddram' or 'bcram' token in filename
my $config;
$config = "sram" if ($file =~ m/.*sram.*/i);
$config = "psram" if ($file =~ m/.*psram.*/i);
$config = "sdram" if ($file =~ m/.*sdram.*/i);
$config = "ddram" if ($file =~ m/.*ddram.*/i);
$config = "bcram" if ($file =~ m/.*bcram.*/i);


# Find board basename
my $board = join('', map {$_ if (index($file, $_) != -1)} (keys %$scripts));
#print "Config: $config\n";
#print "Board: $board\n",(keys %$scripts);

my $pre_load_script = "";
if (exists $scripts->{$board}->{$config}->{'pre'}) {
    $pre_load_script = $scripts->{$board}->{$config}->{'pre'};
}

my $post_load_script = "";
if (exists $scripts->{$board}->{$config}->{'post'}) {
    $post_load_script = $scripts->{$board}->{$config}->{'post'};
}

# Create command file to define "reset" command
open(CMD, ">cmd.gdb") or die("Could not create command file:\n$!");
print(CMD "define reset\n");
print(CMD "    target remote localhost:2331\n");
print(CMD "    monitor reset\n");
print(CMD "    source $pre_load_script\n") if ($pre_load_script ne "");
print(CMD "    load\n");
print(CMD "    source $post_load_script\n") if ($post_load_script ne "");
print(CMD "end");
close(CMD);

# Launch GDB
$pid = fork();
if ($pid == 0) {

   exec("arm-none-eabi-gdb -x cmd.gdb -ex \"reset\" $file");
}
else {

   $SIG{INT} = 'IGNORE';
   $res = waitpid($pid, 0);
}
print("Done\n");
#unlink("cmd.gdb");
