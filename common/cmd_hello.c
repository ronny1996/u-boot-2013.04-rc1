/*
        Command : hello
        function : say hello
*/

/*
 * Boot support
 */
#include <command.h>
#include <common.h>
//#include <linux/compiler.h>

static int do_hello(cmd_tbl_t *cmdtmp, int flag, int argc, char *argv[]) {
  int i = 0;
  printf("hello\n");
  for (i = 0; i < argc; i++) {
    printf("argv[%d] = %s\n", i, argv[i]);
  }
  return 0;
}
/* -------------------------------------------------------------------- */
// register command
/*
#define U_BOOT_CMD_MKENT_COMPLETE(_name, _maxargs, _rep, _cmd,		\
                                _usage, _help, _comp)			\
                { #_name, _maxargs, _rep, _cmd, _usage,			\
                        _CMD_HELP(_help) _CMD_COMPLETE(_comp) }

#define U_BOOT_CMD_MKENT(_name, _maxargs, _rep, _cmd, _usage, _help)	\
        U_BOOT_CMD_MKENT_COMPLETE(_name, _maxargs, _rep, _cmd,		\
                                        _usage, _help, NULL)

#define U_BOOT_CMD_COMPLETE(_name, _maxargs, _rep, _cmd, _usage, _help, _comp) \
        ll_entry_declare(cmd_tbl_t, _name, cmd, cmd) =			\
                U_BOOT_CMD_MKENT_COMPLETE(_name, _maxargs, _rep, _cmd,	\
                                                _usage, _help, _comp);

#define U_BOOT_CMD(_name, _maxargs, _rep, _cmd, _usage, _help)		\
        U_BOOT_CMD_COMPLETE(_name, _maxargs, _rep, _cmd, _usage, _help, NULL)

#define ll_entry_declare(_type, _name, _section_u, _section_d)		\
        _type _u_boot_list_##_section_u##_##_name __attribute__((	\
                        unused,	aligned(4),				\
                        section(".u_boot_list."#_section_d"."#_name)))
*/
U_BOOT_CMD(hello, 10, 1, do_hello, "say hello", "");

/*
        eq :
        U_BOOT_CMD_COMPLETE(hello,1,1,do_hello, "say hello",0,0);
        eq :
        ll_entry_declare(cmd_tbl_t,hello, cmd, cmd) =
   U_BOOT_CMD_MKENT_COMPLETE(hello,1,1,do_hello,"say hello",0,0); eq :
        ll_entry_declare(cmd_tbl_t,hello, cmd, cmd) = {"hello",1,1,do_hello,"say
   hello",0,0}; eq : cmd_tbl_t _u_boot_list_cmd_hello
   __attribute__((unused,aligned(4),section(".u_boot_list.cmd.hello")))
*/
