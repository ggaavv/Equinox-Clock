#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ${OBJECTDIR}/_ext/1472/custom_http_app.o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ${OBJECTDIR}/_ext/1472/ping_demo.o ${OBJECTDIR}/_ext/1472/smtp_demo.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ${OBJECTDIR}/_ext/365611741/system_services.o ${OBJECTDIR}/_ext/1472/main_demo.o ${OBJECTDIR}/_ext/1472/wf_config.o ${OBJECTDIR}/_ext/427700826/dnss.o ${OBJECTDIR}/_ext/427700826/announce.o ${OBJECTDIR}/_ext/427700826/arcfour.o ${OBJECTDIR}/_ext/427700826/arp.o ${OBJECTDIR}/_ext/427700826/auto_ip.o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ${OBJECTDIR}/_ext/427700826/udp_performance_test.o ${OBJECTDIR}/_ext/427700826/dhcp.o ${OBJECTDIR}/_ext/427700826/dhcps.o ${OBJECTDIR}/_ext/427700826/dns.o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ${OBJECTDIR}/_ext/427700826/enc28j60.o ${OBJECTDIR}/_ext/427700826/encs24j600.o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ${OBJECTDIR}/_ext/427700826/file_system.o ${OBJECTDIR}/_ext/427700826/ftp.o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ${OBJECTDIR}/_ext/427700826/http2.o ${OBJECTDIR}/_ext/427700826/icmp.o ${OBJECTDIR}/_ext/427700826/icmpv6.o ${OBJECTDIR}/_ext/427700826/ip.o ${OBJECTDIR}/_ext/427700826/mpfs2.o ${OBJECTDIR}/_ext/427700826/nbns.o ${OBJECTDIR}/_ext/427700826/ndp.o ${OBJECTDIR}/_ext/427700826/reboot.o ${OBJECTDIR}/_ext/427700826/rsa.o ${OBJECTDIR}/_ext/427700826/smtp.o ${OBJECTDIR}/_ext/427700826/snmp.o ${OBJECTDIR}/_ext/427700826/snmpv3.o ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o ${OBJECTDIR}/_ext/427700826/sntp.o ${OBJECTDIR}/_ext/427700826/spi_eeprom.o ${OBJECTDIR}/_ext/427700826/spi_flash.o ${OBJECTDIR}/_ext/427700826/spi_ram.o ${OBJECTDIR}/_ext/427700826/ssl.o ${OBJECTDIR}/_ext/427700826/tcp.o ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ${OBJECTDIR}/_ext/427700826/telnet.o ${OBJECTDIR}/_ext/427700826/tftpc.o ${OBJECTDIR}/_ext/427700826/udp.o ${OBJECTDIR}/_ext/334706090/wf_tx_power.o ${OBJECTDIR}/_ext/334706090/wf_connect.o ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o ${OBJECTDIR}/_ext/334706090/wf_console.o ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o ${OBJECTDIR}/_ext/334706090/wf_driver_com.o ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o ${OBJECTDIR}/_ext/334706090/wf_easy_config.o ${OBJECTDIR}/_ext/334706090/wf_eint.o ${OBJECTDIR}/_ext/334706090/wf_event_handler.o ${OBJECTDIR}/_ext/334706090/wf_init.o ${OBJECTDIR}/_ext/334706090/wf_mac.o ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o ${OBJECTDIR}/_ext/334706090/wf_param_msg.o ${OBJECTDIR}/_ext/334706090/wf_power_save.o ${OBJECTDIR}/_ext/334706090/wf_scan.o ${OBJECTDIR}/_ext/334706090/wf_spi.o ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o ${OBJECTDIR}/_ext/334706090/mrf24w_events.o ${OBJECTDIR}/_ext/101875047/hashes.o ${OBJECTDIR}/_ext/101875047/lfsr.o ${OBJECTDIR}/_ext/365611741/system_random.o ${OBJECTDIR}/_ext/792872985/lcd.o ${OBJECTDIR}/_ext/792872985/usart.o ${OBJECTDIR}/_ext/101875047/big_int.o ${OBJECTDIR}/_ext/101875047/big_int_helper.o ${OBJECTDIR}/_ext/365611741/system_debug.o ${OBJECTDIR}/_ext/792872985/drv_spi.o ${OBJECTDIR}/_ext/999769920/bsp.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d ${OBJECTDIR}/_ext/1472/custom_http_app.o.d ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d ${OBJECTDIR}/_ext/1472/ping_demo.o.d ${OBJECTDIR}/_ext/1472/smtp_demo.o.d ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d ${OBJECTDIR}/_ext/365611741/system_services.o.d ${OBJECTDIR}/_ext/1472/main_demo.o.d ${OBJECTDIR}/_ext/1472/wf_config.o.d ${OBJECTDIR}/_ext/427700826/dnss.o.d ${OBJECTDIR}/_ext/427700826/announce.o.d ${OBJECTDIR}/_ext/427700826/arcfour.o.d ${OBJECTDIR}/_ext/427700826/arp.o.d ${OBJECTDIR}/_ext/427700826/auto_ip.o.d ${OBJECTDIR}/_ext/427700826/berkeley_api.o.d ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d ${OBJECTDIR}/_ext/427700826/dhcp.o.d ${OBJECTDIR}/_ext/427700826/dhcps.o.d ${OBJECTDIR}/_ext/427700826/dns.o.d ${OBJECTDIR}/_ext/427700826/dyn_dns.o.d ${OBJECTDIR}/_ext/427700826/enc28_mac.o.d ${OBJECTDIR}/_ext/427700826/enc28j60.o.d ${OBJECTDIR}/_ext/427700826/encs24j600.o.d ${OBJECTDIR}/_ext/427700826/encx24_mac.o.d ${OBJECTDIR}/_ext/427700826/file_system.o.d ${OBJECTDIR}/_ext/427700826/ftp.o.d ${OBJECTDIR}/_ext/427700826/hash_fnv.o.d ${OBJECTDIR}/_ext/427700826/hash_tbl.o.d ${OBJECTDIR}/_ext/427700826/http2.o.d ${OBJECTDIR}/_ext/427700826/icmp.o.d ${OBJECTDIR}/_ext/427700826/icmpv6.o.d ${OBJECTDIR}/_ext/427700826/ip.o.d ${OBJECTDIR}/_ext/427700826/mpfs2.o.d ${OBJECTDIR}/_ext/427700826/nbns.o.d ${OBJECTDIR}/_ext/427700826/ndp.o.d ${OBJECTDIR}/_ext/427700826/reboot.o.d ${OBJECTDIR}/_ext/427700826/rsa.o.d ${OBJECTDIR}/_ext/427700826/smtp.o.d ${OBJECTDIR}/_ext/427700826/snmp.o.d ${OBJECTDIR}/_ext/427700826/snmpv3.o.d ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d ${OBJECTDIR}/_ext/427700826/sntp.o.d ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d ${OBJECTDIR}/_ext/427700826/spi_flash.o.d ${OBJECTDIR}/_ext/427700826/spi_ram.o.d ${OBJECTDIR}/_ext/427700826/ssl.o.d ${OBJECTDIR}/_ext/427700826/tcp.o.d ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d ${OBJECTDIR}/_ext/427700826/telnet.o.d ${OBJECTDIR}/_ext/427700826/tftpc.o.d ${OBJECTDIR}/_ext/427700826/udp.o.d ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d ${OBJECTDIR}/_ext/334706090/wf_connect.o.d ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d ${OBJECTDIR}/_ext/334706090/wf_console.o.d ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.d ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.d ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.d ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.d ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.d ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.d ${OBJECTDIR}/_ext/334706090/wf_driver_com.o.d ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.d ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d ${OBJECTDIR}/_ext/334706090/wf_eint.o.d ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d ${OBJECTDIR}/_ext/334706090/wf_init.o.d ${OBJECTDIR}/_ext/334706090/wf_mac.o.d ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.d ${OBJECTDIR}/_ext/334706090/wf_param_msg.o.d ${OBJECTDIR}/_ext/334706090/wf_power_save.o.d ${OBJECTDIR}/_ext/334706090/wf_scan.o.d ${OBJECTDIR}/_ext/334706090/wf_spi.o.d ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d ${OBJECTDIR}/_ext/101875047/hashes.o.d ${OBJECTDIR}/_ext/101875047/lfsr.o.d ${OBJECTDIR}/_ext/365611741/system_random.o.d ${OBJECTDIR}/_ext/792872985/lcd.o.d ${OBJECTDIR}/_ext/792872985/usart.o.d ${OBJECTDIR}/_ext/101875047/big_int.o.d ${OBJECTDIR}/_ext/101875047/big_int_helper.o.d ${OBJECTDIR}/_ext/365611741/system_debug.o.d ${OBJECTDIR}/_ext/792872985/drv_spi.o.d ${OBJECTDIR}/_ext/999769920/bsp.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ${OBJECTDIR}/_ext/1472/custom_http_app.o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ${OBJECTDIR}/_ext/1472/ping_demo.o ${OBJECTDIR}/_ext/1472/smtp_demo.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ${OBJECTDIR}/_ext/365611741/system_services.o ${OBJECTDIR}/_ext/1472/main_demo.o ${OBJECTDIR}/_ext/1472/wf_config.o ${OBJECTDIR}/_ext/427700826/dnss.o ${OBJECTDIR}/_ext/427700826/announce.o ${OBJECTDIR}/_ext/427700826/arcfour.o ${OBJECTDIR}/_ext/427700826/arp.o ${OBJECTDIR}/_ext/427700826/auto_ip.o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ${OBJECTDIR}/_ext/427700826/udp_performance_test.o ${OBJECTDIR}/_ext/427700826/dhcp.o ${OBJECTDIR}/_ext/427700826/dhcps.o ${OBJECTDIR}/_ext/427700826/dns.o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ${OBJECTDIR}/_ext/427700826/enc28j60.o ${OBJECTDIR}/_ext/427700826/encs24j600.o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ${OBJECTDIR}/_ext/427700826/file_system.o ${OBJECTDIR}/_ext/427700826/ftp.o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ${OBJECTDIR}/_ext/427700826/http2.o ${OBJECTDIR}/_ext/427700826/icmp.o ${OBJECTDIR}/_ext/427700826/icmpv6.o ${OBJECTDIR}/_ext/427700826/ip.o ${OBJECTDIR}/_ext/427700826/mpfs2.o ${OBJECTDIR}/_ext/427700826/nbns.o ${OBJECTDIR}/_ext/427700826/ndp.o ${OBJECTDIR}/_ext/427700826/reboot.o ${OBJECTDIR}/_ext/427700826/rsa.o ${OBJECTDIR}/_ext/427700826/smtp.o ${OBJECTDIR}/_ext/427700826/snmp.o ${OBJECTDIR}/_ext/427700826/snmpv3.o ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o ${OBJECTDIR}/_ext/427700826/sntp.o ${OBJECTDIR}/_ext/427700826/spi_eeprom.o ${OBJECTDIR}/_ext/427700826/spi_flash.o ${OBJECTDIR}/_ext/427700826/spi_ram.o ${OBJECTDIR}/_ext/427700826/ssl.o ${OBJECTDIR}/_ext/427700826/tcp.o ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ${OBJECTDIR}/_ext/427700826/telnet.o ${OBJECTDIR}/_ext/427700826/tftpc.o ${OBJECTDIR}/_ext/427700826/udp.o ${OBJECTDIR}/_ext/334706090/wf_tx_power.o ${OBJECTDIR}/_ext/334706090/wf_connect.o ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o ${OBJECTDIR}/_ext/334706090/wf_console.o ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o ${OBJECTDIR}/_ext/334706090/wf_driver_com.o ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o ${OBJECTDIR}/_ext/334706090/wf_easy_config.o ${OBJECTDIR}/_ext/334706090/wf_eint.o ${OBJECTDIR}/_ext/334706090/wf_event_handler.o ${OBJECTDIR}/_ext/334706090/wf_init.o ${OBJECTDIR}/_ext/334706090/wf_mac.o ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o ${OBJECTDIR}/_ext/334706090/wf_param_msg.o ${OBJECTDIR}/_ext/334706090/wf_power_save.o ${OBJECTDIR}/_ext/334706090/wf_scan.o ${OBJECTDIR}/_ext/334706090/wf_spi.o ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o ${OBJECTDIR}/_ext/334706090/mrf24w_events.o ${OBJECTDIR}/_ext/101875047/hashes.o ${OBJECTDIR}/_ext/101875047/lfsr.o ${OBJECTDIR}/_ext/365611741/system_random.o ${OBJECTDIR}/_ext/792872985/lcd.o ${OBJECTDIR}/_ext/792872985/usart.o ${OBJECTDIR}/_ext/101875047/big_int.o ${OBJECTDIR}/_ext/101875047/big_int_helper.o ${OBJECTDIR}/_ext/365611741/system_debug.o ${OBJECTDIR}/_ext/792872985/drv_spi.o ${OBJECTDIR}/_ext/999769920/bsp.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

# The following macros may be used in the pre and post step lines
Device=PIC24FJ256GB110
ProjectDir="C:\mk\dev\cleanCheck\tcpip\tcpip_web_server_demo_app\tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X"
ConfName=default
ImagePath="dist\default\${IMAGE_TYPE}\tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}"
ImageDir="dist\default\${IMAGE_TYPE}"
ImageName="tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}"

.build-conf:  .pre ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
	@echo "--------------------------------------"
	@echo "User defined post-build step: []"
	@
	@echo "--------------------------------------"

MP_PROCESSOR_OPTION=24FJ256GB110
MP_LINKER_FILE_OPTION=,-Tp24FJ256GB110.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/101875047/big_int_helper.o: ../../../microchip/common/big_int_helper.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper.o.d ${OBJECTDIR}/_ext/101875047/big_int_helper.o.asm.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper.o.ok ${OBJECTDIR}/_ext/101875047/big_int_helper.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int_helper.o.d" "${OBJECTDIR}/_ext/101875047/big_int_helper.o.asm.d" -t $(SILENT) -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -c -mcpu=$(MP_PROCESSOR_OPTION) -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int_helper.o.d"  -o ${OBJECTDIR}/_ext/101875047/big_int_helper.o ../../../microchip/common/big_int_helper.S  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/101875047/big_int_helper.o.asm.d",--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-g,-I"..",-g 
	
else
${OBJECTDIR}/_ext/101875047/big_int_helper.o: ../../../microchip/common/big_int_helper.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper.o.d ${OBJECTDIR}/_ext/101875047/big_int_helper.o.asm.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper.o.ok ${OBJECTDIR}/_ext/101875047/big_int_helper.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int_helper.o.d" "${OBJECTDIR}/_ext/101875047/big_int_helper.o.asm.d" -t $(SILENT) -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -omf=elf -c -mcpu=$(MP_PROCESSOR_OPTION) -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int_helper.o.d"  -o ${OBJECTDIR}/_ext/101875047/big_int_helper.o ../../../microchip/common/big_int_helper.S  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/101875047/big_int_helper.o.asm.d",-g,-I"..",-g 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/custom_ssl_cert.o: ../custom_ssl_cert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.ok ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" -o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ../custom_ssl_cert.c  
	
${OBJECTDIR}/_ext/1472/custom_http_app.o: ../custom_http_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o.ok ${OBJECTDIR}/_ext/1472/custom_http_app.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_http_app.o ../custom_http_app.c  
	
${OBJECTDIR}/_ext/1472/custom_snmp_app.o: ../custom_snmp_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.ok ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ../custom_snmp_app.c  
	
${OBJECTDIR}/_ext/1472/ping_demo.o: ../ping_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o.ok ${OBJECTDIR}/_ext/1472/ping_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ping_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/ping_demo.o.d" -o ${OBJECTDIR}/_ext/1472/ping_demo.o ../ping_demo.c  
	
${OBJECTDIR}/_ext/1472/smtp_demo.o: ../smtp_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o.ok ${OBJECTDIR}/_ext/1472/smtp_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" -o ${OBJECTDIR}/_ext/1472/smtp_demo.o ../smtp_demo.c  
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o: ../berkeley_tcp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.ok ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ../berkeley_tcp_client_demo.c  
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o: ../berkeley_tcp_server_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.ok ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ../berkeley_tcp_server_demo.c  
	
${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o: ../berkeley_udp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.ok ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ../berkeley_udp_client_demo.c  
	
${OBJECTDIR}/_ext/1472/generic_tcp_client.o: ../generic_tcp_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.ok ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ../generic_tcp_client.c  
	
${OBJECTDIR}/_ext/1472/generic_tcp_server.o: ../generic_tcp_server.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.ok ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ../generic_tcp_server.c  
	
${OBJECTDIR}/_ext/365611741/system_services.o: ../../../microchip/system/system_services.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o.ok ${OBJECTDIR}/_ext/365611741/system_services.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_services.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624 -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/365611741/system_services.o.d" -o ${OBJECTDIR}/_ext/365611741/system_services.o ../../../microchip/system/system_services.c  
	
${OBJECTDIR}/_ext/1472/main_demo.o: ../main_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o.ok ${OBJECTDIR}/_ext/1472/main_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/main_demo.o.d" -o ${OBJECTDIR}/_ext/1472/main_demo.o ../main_demo.c  
	
${OBJECTDIR}/_ext/1472/wf_config.o: ../wf_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/wf_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/wf_config.o.ok ${OBJECTDIR}/_ext/1472/wf_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/wf_config.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/wf_config.o.d" -o ${OBJECTDIR}/_ext/1472/wf_config.o ../wf_config.c  
	
${OBJECTDIR}/_ext/427700826/dnss.o: ../../../microchip/tcpip/dnss.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o.ok ${OBJECTDIR}/_ext/427700826/dnss.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dnss.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dnss.o.d" -o ${OBJECTDIR}/_ext/427700826/dnss.o ../../../microchip/tcpip/dnss.c  
	
${OBJECTDIR}/_ext/427700826/announce.o: ../../../microchip/tcpip/announce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/announce.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/announce.o.ok ${OBJECTDIR}/_ext/427700826/announce.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/announce.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/announce.o.d" -o ${OBJECTDIR}/_ext/427700826/announce.o ../../../microchip/tcpip/announce.c  
	
${OBJECTDIR}/_ext/427700826/arcfour.o: ../../../microchip/tcpip/arcfour.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o.ok ${OBJECTDIR}/_ext/427700826/arcfour.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arcfour.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/arcfour.o.d" -o ${OBJECTDIR}/_ext/427700826/arcfour.o ../../../microchip/tcpip/arcfour.c  
	
${OBJECTDIR}/_ext/427700826/arp.o: ../../../microchip/tcpip/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o.ok ${OBJECTDIR}/_ext/427700826/arp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/arp.o.d" -o ${OBJECTDIR}/_ext/427700826/arp.o ../../../microchip/tcpip/arp.c  
	
${OBJECTDIR}/_ext/427700826/auto_ip.o: ../../../microchip/tcpip/auto_ip.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/auto_ip.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/auto_ip.o.ok ${OBJECTDIR}/_ext/427700826/auto_ip.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/auto_ip.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/auto_ip.o.d" -o ${OBJECTDIR}/_ext/427700826/auto_ip.o ../../../microchip/tcpip/auto_ip.c  
	
${OBJECTDIR}/_ext/427700826/berkeley_api.o: ../../../microchip/tcpip/berkeley_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o.ok ${OBJECTDIR}/_ext/427700826/berkeley_api.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" -o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ../../../microchip/tcpip/berkeley_api.c  
	
${OBJECTDIR}/_ext/427700826/udp_performance_test.o: ../../../microchip/tcpip/udp_performance_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.ok ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d" -o ${OBJECTDIR}/_ext/427700826/udp_performance_test.o ../../../microchip/tcpip/udp_performance_test.c  
	
${OBJECTDIR}/_ext/427700826/dhcp.o: ../../../microchip/tcpip/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o.ok ${OBJECTDIR}/_ext/427700826/dhcp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcp.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcp.o ../../../microchip/tcpip/dhcp.c  
	
${OBJECTDIR}/_ext/427700826/dhcps.o: ../../../microchip/tcpip/dhcps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o.ok ${OBJECTDIR}/_ext/427700826/dhcps.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcps.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcps.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcps.o ../../../microchip/tcpip/dhcps.c  
	
${OBJECTDIR}/_ext/427700826/dns.o: ../../../microchip/tcpip/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o.ok ${OBJECTDIR}/_ext/427700826/dns.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dns.o ../../../microchip/tcpip/dns.c  
	
${OBJECTDIR}/_ext/427700826/dyn_dns.o: ../../../microchip/tcpip/dyn_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o.ok ${OBJECTDIR}/_ext/427700826/dyn_dns.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ../../../microchip/tcpip/dyn_dns.c  
	
${OBJECTDIR}/_ext/427700826/enc28_mac.o: ../../../microchip/tcpip/enc28_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o.ok ${OBJECTDIR}/_ext/427700826/enc28_mac.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ../../../microchip/tcpip/enc28_mac.c  
	
${OBJECTDIR}/_ext/427700826/enc28j60.o: ../../../microchip/tcpip/enc28j60.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o.ok ${OBJECTDIR}/_ext/427700826/enc28j60.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28j60.o ../../../microchip/tcpip/enc28j60.c  
	
${OBJECTDIR}/_ext/427700826/encs24j600.o: ../../../microchip/tcpip/encs24j600.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o.ok ${OBJECTDIR}/_ext/427700826/encs24j600.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" -o ${OBJECTDIR}/_ext/427700826/encs24j600.o ../../../microchip/tcpip/encs24j600.c  
	
${OBJECTDIR}/_ext/427700826/encx24_mac.o: ../../../microchip/tcpip/encx24_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o.ok ${OBJECTDIR}/_ext/427700826/encx24_mac.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ../../../microchip/tcpip/encx24_mac.c  
	
${OBJECTDIR}/_ext/427700826/file_system.o: ../../../microchip/tcpip/file_system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/file_system.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/file_system.o.ok ${OBJECTDIR}/_ext/427700826/file_system.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/file_system.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/file_system.o.d" -o ${OBJECTDIR}/_ext/427700826/file_system.o ../../../microchip/tcpip/file_system.c  
	
${OBJECTDIR}/_ext/427700826/ftp.o: ../../../microchip/tcpip/ftp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o.ok ${OBJECTDIR}/_ext/427700826/ftp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ftp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/ftp.o.d" -o ${OBJECTDIR}/_ext/427700826/ftp.o ../../../microchip/tcpip/ftp.c  
	
${OBJECTDIR}/_ext/427700826/hash_fnv.o: ../../../microchip/tcpip/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o.ok ${OBJECTDIR}/_ext/427700826/hash_fnv.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ../../../microchip/tcpip/hash_fnv.c  
	
${OBJECTDIR}/_ext/427700826/hash_tbl.o: ../../../microchip/tcpip/hash_tbl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o.ok ${OBJECTDIR}/_ext/427700826/hash_tbl.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ../../../microchip/tcpip/hash_tbl.c  
	
${OBJECTDIR}/_ext/427700826/http2.o: ../../../microchip/tcpip/http2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o.ok ${OBJECTDIR}/_ext/427700826/http2.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/http2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/http2.o.d" -o ${OBJECTDIR}/_ext/427700826/http2.o ../../../microchip/tcpip/http2.c  
	
${OBJECTDIR}/_ext/427700826/icmp.o: ../../../microchip/tcpip/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o.ok ${OBJECTDIR}/_ext/427700826/icmp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/icmp.o.d" -o ${OBJECTDIR}/_ext/427700826/icmp.o ../../../microchip/tcpip/icmp.c  
	
${OBJECTDIR}/_ext/427700826/icmpv6.o: ../../../microchip/tcpip/icmpv6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o.ok ${OBJECTDIR}/_ext/427700826/icmpv6.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" -o ${OBJECTDIR}/_ext/427700826/icmpv6.o ../../../microchip/tcpip/icmpv6.c  
	
${OBJECTDIR}/_ext/427700826/ip.o: ../../../microchip/tcpip/ip.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ip.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ip.o.ok ${OBJECTDIR}/_ext/427700826/ip.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ip.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/ip.o.d" -o ${OBJECTDIR}/_ext/427700826/ip.o ../../../microchip/tcpip/ip.c  
	
${OBJECTDIR}/_ext/427700826/mpfs2.o: ../../../microchip/tcpip/mpfs2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/mpfs2.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/mpfs2.o.ok ${OBJECTDIR}/_ext/427700826/mpfs2.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/mpfs2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/mpfs2.o.d" -o ${OBJECTDIR}/_ext/427700826/mpfs2.o ../../../microchip/tcpip/mpfs2.c  
	
${OBJECTDIR}/_ext/427700826/nbns.o: ../../../microchip/tcpip/nbns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o.ok ${OBJECTDIR}/_ext/427700826/nbns.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/nbns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/nbns.o.d" -o ${OBJECTDIR}/_ext/427700826/nbns.o ../../../microchip/tcpip/nbns.c  
	
${OBJECTDIR}/_ext/427700826/ndp.o: ../../../microchip/tcpip/ndp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o.ok ${OBJECTDIR}/_ext/427700826/ndp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ndp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/ndp.o.d" -o ${OBJECTDIR}/_ext/427700826/ndp.o ../../../microchip/tcpip/ndp.c  
	
${OBJECTDIR}/_ext/427700826/reboot.o: ../../../microchip/tcpip/reboot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/reboot.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/reboot.o.ok ${OBJECTDIR}/_ext/427700826/reboot.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/reboot.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/reboot.o.d" -o ${OBJECTDIR}/_ext/427700826/reboot.o ../../../microchip/tcpip/reboot.c  
	
${OBJECTDIR}/_ext/427700826/rsa.o: ../../../microchip/tcpip/rsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o.ok ${OBJECTDIR}/_ext/427700826/rsa.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/rsa.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/rsa.o.d" -o ${OBJECTDIR}/_ext/427700826/rsa.o ../../../microchip/tcpip/rsa.c  
	
${OBJECTDIR}/_ext/427700826/smtp.o: ../../../microchip/tcpip/smtp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o.ok ${OBJECTDIR}/_ext/427700826/smtp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/smtp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/smtp.o.d" -o ${OBJECTDIR}/_ext/427700826/smtp.o ../../../microchip/tcpip/smtp.c  
	
${OBJECTDIR}/_ext/427700826/snmp.o: ../../../microchip/tcpip/snmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o.ok ${OBJECTDIR}/_ext/427700826/snmp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/snmp.o.d" -o ${OBJECTDIR}/_ext/427700826/snmp.o ../../../microchip/tcpip/snmp.c  
	
${OBJECTDIR}/_ext/427700826/snmpv3.o: ../../../microchip/tcpip/snmpv3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3.o.ok ${OBJECTDIR}/_ext/427700826/snmpv3.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmpv3.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/snmpv3.o.d" -o ${OBJECTDIR}/_ext/427700826/snmpv3.o ../../../microchip/tcpip/snmpv3.c  
	
${OBJECTDIR}/_ext/427700826/snmpv3_usm.o: ../../../microchip/tcpip/snmpv3_usm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.ok ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d" -o ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o ../../../microchip/tcpip/snmpv3_usm.c  
	
${OBJECTDIR}/_ext/427700826/sntp.o: ../../../microchip/tcpip/sntp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o.ok ${OBJECTDIR}/_ext/427700826/sntp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/sntp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/sntp.o.d" -o ${OBJECTDIR}/_ext/427700826/sntp.o ../../../microchip/tcpip/sntp.c  
	
${OBJECTDIR}/_ext/427700826/spi_eeprom.o: ../../../microchip/tcpip/spi_eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.ok ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_eeprom.o ../../../microchip/tcpip/spi_eeprom.c  
	
${OBJECTDIR}/_ext/427700826/spi_flash.o: ../../../microchip/tcpip/spi_flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_flash.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_flash.o.ok ${OBJECTDIR}/_ext/427700826/spi_flash.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_flash.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_flash.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_flash.o ../../../microchip/tcpip/spi_flash.c  
	
${OBJECTDIR}/_ext/427700826/spi_ram.o: ../../../microchip/tcpip/spi_ram.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_ram.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_ram.o.ok ${OBJECTDIR}/_ext/427700826/spi_ram.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_ram.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_ram.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_ram.o ../../../microchip/tcpip/spi_ram.c  
	
${OBJECTDIR}/_ext/427700826/ssl.o: ../../../microchip/tcpip/ssl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o.ok ${OBJECTDIR}/_ext/427700826/ssl.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ssl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/ssl.o.d" -o ${OBJECTDIR}/_ext/427700826/ssl.o ../../../microchip/tcpip/ssl.c  
	
${OBJECTDIR}/_ext/427700826/tcp.o: ../../../microchip/tcpip/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o.ok ${OBJECTDIR}/_ext/427700826/tcp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp.o ../../../microchip/tcpip/tcp.c  
	
${OBJECTDIR}/_ext/427700826/tcp_performance_test.o: ../../../microchip/tcpip/tcp_performance_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.ok ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o ../../../microchip/tcpip/tcp_performance_test.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o: ../../../microchip/tcpip/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ../../../microchip/tcpip/tcpip_heap_alloc.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_helpers.o: ../../../microchip/tcpip/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ../../../microchip/tcpip/tcpip_helpers.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o: ../../../microchip/tcpip/tcpip_mac_object.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624 -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ../../../microchip/tcpip/tcpip_mac_object.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_manager.o: ../../../microchip/tcpip/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ../../../microchip/tcpip/tcpip_manager.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_storage.o: ../../../microchip/tcpip/tcpip_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ../../../microchip/tcpip/tcpip_storage.c  
	
${OBJECTDIR}/_ext/427700826/telnet.o: ../../../microchip/tcpip/telnet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o.ok ${OBJECTDIR}/_ext/427700826/telnet.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/telnet.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/telnet.o.d" -o ${OBJECTDIR}/_ext/427700826/telnet.o ../../../microchip/tcpip/telnet.c  
	
${OBJECTDIR}/_ext/427700826/tftpc.o: ../../../microchip/tcpip/tftpc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o.ok ${OBJECTDIR}/_ext/427700826/tftpc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tftpc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tftpc.o.d" -o ${OBJECTDIR}/_ext/427700826/tftpc.o ../../../microchip/tcpip/tftpc.c  
	
${OBJECTDIR}/_ext/427700826/udp.o: ../../../microchip/tcpip/udp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o.ok ${OBJECTDIR}/_ext/427700826/udp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/udp.o.d" -o ${OBJECTDIR}/_ext/427700826/udp.o ../../../microchip/tcpip/udp.c  
	
${OBJECTDIR}/_ext/334706090/wf_tx_power.o: ../../../microchip/tcpip/wifi/wf_tx_power.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.ok ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_tx_power.o ../../../microchip/tcpip/wifi/wf_tx_power.c  
	
${OBJECTDIR}/_ext/334706090/wf_connect.o: ../../../microchip/tcpip/wifi/wf_connect.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connect.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connect.o.ok ${OBJECTDIR}/_ext/334706090/wf_connect.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connect.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connect.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connect.o ../../../microchip/tcpip/wifi/wf_connect.c  
	
${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o: ../../../microchip/tcpip/wifi/wf_connection_algorithm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.ok ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o ../../../microchip/tcpip/wifi/wf_connection_algorithm.c  
	
${OBJECTDIR}/_ext/334706090/wf_connection_manager.o: ../../../microchip/tcpip/wifi/wf_connection_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.ok ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o ../../../microchip/tcpip/wifi/wf_connection_manager.c  
	
${OBJECTDIR}/_ext/334706090/wf_connection_profile.o: ../../../microchip/tcpip/wifi/wf_connection_profile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.ok ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o ../../../microchip/tcpip/wifi/wf_connection_profile.c  
	
${OBJECTDIR}/_ext/334706090/wf_console.o: ../../../microchip/tcpip/wifi/wf_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console.o.ok ${OBJECTDIR}/_ext/334706090/wf_console.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console.o ../../../microchip/tcpip/wifi/wf_console.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_if_config.o: ../../../microchip/tcpip/wifi/wf_console_if_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o ../../../microchip/tcpip/wifi/wf_console_if_config.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o: ../../../microchip/tcpip/wifi/wf_console_iw_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o ../../../microchip/tcpip/wifi/wf_console_iw_config.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o: ../../../microchip/tcpip/wifi/wf_console_iw_priv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o ../../../microchip/tcpip/wifi/wf_console_iw_priv.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o: ../../../microchip/tcpip/wifi/wf_console_msg_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o ../../../microchip/tcpip/wifi/wf_console_msg_handler.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_msgs.o: ../../../microchip/tcpip/wifi/wf_console_msgs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o ../../../microchip/tcpip/wifi/wf_console_msgs.c  
	
${OBJECTDIR}/_ext/334706090/wf_data_txrx.o: ../../../microchip/tcpip/wifi/wf_data_txrx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.ok ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o ../../../microchip/tcpip/wifi/wf_data_txrx.c  
	
${OBJECTDIR}/_ext/334706090/wf_driver_com.o: ../../../microchip/tcpip/wifi/wf_driver_com.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_com.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_com.o.ok ${OBJECTDIR}/_ext/334706090/wf_driver_com.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_driver_com.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_driver_com.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_driver_com.o ../../../microchip/tcpip/wifi/wf_driver_com.c  
	
${OBJECTDIR}/_ext/334706090/wf_driver_raw.o: ../../../microchip/tcpip/wifi/wf_driver_raw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.ok ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o ../../../microchip/tcpip/wifi/wf_driver_raw.c  
	
${OBJECTDIR}/_ext/334706090/wf_easy_config.o: ../../../microchip/tcpip/wifi/wf_easy_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.ok ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_easy_config.o ../../../microchip/tcpip/wifi/wf_easy_config.c  
	
${OBJECTDIR}/_ext/334706090/wf_eint.o: ../../../microchip/tcpip/wifi/wf_eint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_eint.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_eint.o.ok ${OBJECTDIR}/_ext/334706090/wf_eint.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_eint.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_eint.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_eint.o ../../../microchip/tcpip/wifi/wf_eint.c  
	
${OBJECTDIR}/_ext/334706090/wf_event_handler.o: ../../../microchip/tcpip/wifi/wf_event_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.ok ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_event_handler.o ../../../microchip/tcpip/wifi/wf_event_handler.c  
	
${OBJECTDIR}/_ext/334706090/wf_init.o: ../../../microchip/tcpip/wifi/wf_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_init.o.ok ${OBJECTDIR}/_ext/334706090/wf_init.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_init.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_init.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_init.o ../../../microchip/tcpip/wifi/wf_init.c  
	
${OBJECTDIR}/_ext/334706090/wf_mac.o: ../../../microchip/tcpip/wifi/wf_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mac.o.ok ${OBJECTDIR}/_ext/334706090/wf_mac.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_mac.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_mac.o ../../../microchip/tcpip/wifi/wf_mac.c  
	
${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o: ../../../microchip/tcpip/wifi/wf_mgmt_msg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.ok ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o ../../../microchip/tcpip/wifi/wf_mgmt_msg.c  
	
${OBJECTDIR}/_ext/334706090/wf_param_msg.o: ../../../microchip/tcpip/wifi/wf_param_msg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_param_msg.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_param_msg.o.ok ${OBJECTDIR}/_ext/334706090/wf_param_msg.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_param_msg.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_param_msg.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_param_msg.o ../../../microchip/tcpip/wifi/wf_param_msg.c  
	
${OBJECTDIR}/_ext/334706090/wf_power_save.o: ../../../microchip/tcpip/wifi/wf_power_save.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_power_save.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_power_save.o.ok ${OBJECTDIR}/_ext/334706090/wf_power_save.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_power_save.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_power_save.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_power_save.o ../../../microchip/tcpip/wifi/wf_power_save.c  
	
${OBJECTDIR}/_ext/334706090/wf_scan.o: ../../../microchip/tcpip/wifi/wf_scan.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_scan.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_scan.o.ok ${OBJECTDIR}/_ext/334706090/wf_scan.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_scan.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_scan.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_scan.o ../../../microchip/tcpip/wifi/wf_scan.c  
	
${OBJECTDIR}/_ext/334706090/wf_spi.o: ../../../microchip/tcpip/wifi/wf_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_spi.o.ok ${OBJECTDIR}/_ext/334706090/wf_spi.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_spi.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_spi.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_spi.o ../../../microchip/tcpip/wifi/wf_spi.c  
	
${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o: ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.ok ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d" -o ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c  
	
${OBJECTDIR}/_ext/334706090/mrf24w_events.o: ../../../microchip/tcpip/wifi/mrf24w_events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.ok ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d" -o ${OBJECTDIR}/_ext/334706090/mrf24w_events.o ../../../microchip/tcpip/wifi/mrf24w_events.c  
	
${OBJECTDIR}/_ext/101875047/hashes.o: ../../../microchip/common/hashes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o.ok ${OBJECTDIR}/_ext/101875047/hashes.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/hashes.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/101875047/hashes.o.d" -o ${OBJECTDIR}/_ext/101875047/hashes.o ../../../microchip/common/hashes.c  
	
${OBJECTDIR}/_ext/101875047/lfsr.o: ../../../microchip/common/lfsr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o.ok ${OBJECTDIR}/_ext/101875047/lfsr.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/lfsr.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/101875047/lfsr.o.d" -o ${OBJECTDIR}/_ext/101875047/lfsr.o ../../../microchip/common/lfsr.c  
	
${OBJECTDIR}/_ext/365611741/system_random.o: ../../../microchip/system/system_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o.ok ${OBJECTDIR}/_ext/365611741/system_random.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_random.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/365611741/system_random.o.d" -o ${OBJECTDIR}/_ext/365611741/system_random.o ../../../microchip/system/system_random.c  
	
${OBJECTDIR}/_ext/792872985/lcd.o: ../../../microchip/system/drivers/lcd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o.ok ${OBJECTDIR}/_ext/792872985/lcd.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/lcd.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/792872985/lcd.o.d" -o ${OBJECTDIR}/_ext/792872985/lcd.o ../../../microchip/system/drivers/lcd.c  
	
${OBJECTDIR}/_ext/792872985/usart.o: ../../../microchip/system/drivers/usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o.ok ${OBJECTDIR}/_ext/792872985/usart.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/usart.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/792872985/usart.o.d" -o ${OBJECTDIR}/_ext/792872985/usart.o ../../../microchip/system/drivers/usart.c  
	
${OBJECTDIR}/_ext/101875047/big_int.o: ../../../microchip/common/big_int.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o.ok ${OBJECTDIR}/_ext/101875047/big_int.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int.o.d" -o ${OBJECTDIR}/_ext/101875047/big_int.o ../../../microchip/common/big_int.c  
	
${OBJECTDIR}/_ext/365611741/system_debug.o: ../../../microchip/system/system_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o.ok ${OBJECTDIR}/_ext/365611741/system_debug.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_debug.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/365611741/system_debug.o.d" -o ${OBJECTDIR}/_ext/365611741/system_debug.o ../../../microchip/system/system_debug.c  
	
${OBJECTDIR}/_ext/792872985/drv_spi.o: ../../../microchip/system/drivers/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o.ok ${OBJECTDIR}/_ext/792872985/drv_spi.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" -o ${OBJECTDIR}/_ext/792872985/drv_spi.o ../../../microchip/system/drivers/drv_spi.c  
	
${OBJECTDIR}/_ext/999769920/bsp.o: ../../../bsp/explorer16/pic24/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/999769920 
	@${RM} ${OBJECTDIR}/_ext/999769920/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/999769920/bsp.o.ok ${OBJECTDIR}/_ext/999769920/bsp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/999769920/bsp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/999769920/bsp.o.d" -o ${OBJECTDIR}/_ext/999769920/bsp.o ../../../bsp/explorer16/pic24/bsp.c  
	
else
${OBJECTDIR}/_ext/1472/custom_ssl_cert.o: ../custom_ssl_cert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.ok ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" -o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ../custom_ssl_cert.c  
	
${OBJECTDIR}/_ext/1472/custom_http_app.o: ../custom_http_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o.ok ${OBJECTDIR}/_ext/1472/custom_http_app.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_http_app.o ../custom_http_app.c  
	
${OBJECTDIR}/_ext/1472/custom_snmp_app.o: ../custom_snmp_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.ok ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ../custom_snmp_app.c  
	
${OBJECTDIR}/_ext/1472/ping_demo.o: ../ping_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o.ok ${OBJECTDIR}/_ext/1472/ping_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ping_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/ping_demo.o.d" -o ${OBJECTDIR}/_ext/1472/ping_demo.o ../ping_demo.c  
	
${OBJECTDIR}/_ext/1472/smtp_demo.o: ../smtp_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o.ok ${OBJECTDIR}/_ext/1472/smtp_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" -o ${OBJECTDIR}/_ext/1472/smtp_demo.o ../smtp_demo.c  
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o: ../berkeley_tcp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.ok ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ../berkeley_tcp_client_demo.c  
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o: ../berkeley_tcp_server_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.ok ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ../berkeley_tcp_server_demo.c  
	
${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o: ../berkeley_udp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.ok ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ../berkeley_udp_client_demo.c  
	
${OBJECTDIR}/_ext/1472/generic_tcp_client.o: ../generic_tcp_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.ok ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ../generic_tcp_client.c  
	
${OBJECTDIR}/_ext/1472/generic_tcp_server.o: ../generic_tcp_server.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.ok ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ../generic_tcp_server.c  
	
${OBJECTDIR}/_ext/365611741/system_services.o: ../../../microchip/system/system_services.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o.ok ${OBJECTDIR}/_ext/365611741/system_services.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_services.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624 -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/365611741/system_services.o.d" -o ${OBJECTDIR}/_ext/365611741/system_services.o ../../../microchip/system/system_services.c  
	
${OBJECTDIR}/_ext/1472/main_demo.o: ../main_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o.ok ${OBJECTDIR}/_ext/1472/main_demo.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/main_demo.o.d" -o ${OBJECTDIR}/_ext/1472/main_demo.o ../main_demo.c  
	
${OBJECTDIR}/_ext/1472/wf_config.o: ../wf_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/wf_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/wf_config.o.ok ${OBJECTDIR}/_ext/1472/wf_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/wf_config.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/1472/wf_config.o.d" -o ${OBJECTDIR}/_ext/1472/wf_config.o ../wf_config.c  
	
${OBJECTDIR}/_ext/427700826/dnss.o: ../../../microchip/tcpip/dnss.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o.ok ${OBJECTDIR}/_ext/427700826/dnss.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dnss.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dnss.o.d" -o ${OBJECTDIR}/_ext/427700826/dnss.o ../../../microchip/tcpip/dnss.c  
	
${OBJECTDIR}/_ext/427700826/announce.o: ../../../microchip/tcpip/announce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/announce.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/announce.o.ok ${OBJECTDIR}/_ext/427700826/announce.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/announce.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/announce.o.d" -o ${OBJECTDIR}/_ext/427700826/announce.o ../../../microchip/tcpip/announce.c  
	
${OBJECTDIR}/_ext/427700826/arcfour.o: ../../../microchip/tcpip/arcfour.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o.ok ${OBJECTDIR}/_ext/427700826/arcfour.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arcfour.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/arcfour.o.d" -o ${OBJECTDIR}/_ext/427700826/arcfour.o ../../../microchip/tcpip/arcfour.c  
	
${OBJECTDIR}/_ext/427700826/arp.o: ../../../microchip/tcpip/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o.ok ${OBJECTDIR}/_ext/427700826/arp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/arp.o.d" -o ${OBJECTDIR}/_ext/427700826/arp.o ../../../microchip/tcpip/arp.c  
	
${OBJECTDIR}/_ext/427700826/auto_ip.o: ../../../microchip/tcpip/auto_ip.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/auto_ip.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/auto_ip.o.ok ${OBJECTDIR}/_ext/427700826/auto_ip.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/auto_ip.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/auto_ip.o.d" -o ${OBJECTDIR}/_ext/427700826/auto_ip.o ../../../microchip/tcpip/auto_ip.c  
	
${OBJECTDIR}/_ext/427700826/berkeley_api.o: ../../../microchip/tcpip/berkeley_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o.ok ${OBJECTDIR}/_ext/427700826/berkeley_api.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" -o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ../../../microchip/tcpip/berkeley_api.c  
	
${OBJECTDIR}/_ext/427700826/udp_performance_test.o: ../../../microchip/tcpip/udp_performance_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.ok ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d" -o ${OBJECTDIR}/_ext/427700826/udp_performance_test.o ../../../microchip/tcpip/udp_performance_test.c  
	
${OBJECTDIR}/_ext/427700826/dhcp.o: ../../../microchip/tcpip/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o.ok ${OBJECTDIR}/_ext/427700826/dhcp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcp.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcp.o ../../../microchip/tcpip/dhcp.c  
	
${OBJECTDIR}/_ext/427700826/dhcps.o: ../../../microchip/tcpip/dhcps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o.ok ${OBJECTDIR}/_ext/427700826/dhcps.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcps.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcps.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcps.o ../../../microchip/tcpip/dhcps.c  
	
${OBJECTDIR}/_ext/427700826/dns.o: ../../../microchip/tcpip/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o.ok ${OBJECTDIR}/_ext/427700826/dns.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dns.o ../../../microchip/tcpip/dns.c  
	
${OBJECTDIR}/_ext/427700826/dyn_dns.o: ../../../microchip/tcpip/dyn_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o.ok ${OBJECTDIR}/_ext/427700826/dyn_dns.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ../../../microchip/tcpip/dyn_dns.c  
	
${OBJECTDIR}/_ext/427700826/enc28_mac.o: ../../../microchip/tcpip/enc28_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o.ok ${OBJECTDIR}/_ext/427700826/enc28_mac.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ../../../microchip/tcpip/enc28_mac.c  
	
${OBJECTDIR}/_ext/427700826/enc28j60.o: ../../../microchip/tcpip/enc28j60.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o.ok ${OBJECTDIR}/_ext/427700826/enc28j60.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28j60.o ../../../microchip/tcpip/enc28j60.c  
	
${OBJECTDIR}/_ext/427700826/encs24j600.o: ../../../microchip/tcpip/encs24j600.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o.ok ${OBJECTDIR}/_ext/427700826/encs24j600.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" -o ${OBJECTDIR}/_ext/427700826/encs24j600.o ../../../microchip/tcpip/encs24j600.c  
	
${OBJECTDIR}/_ext/427700826/encx24_mac.o: ../../../microchip/tcpip/encx24_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o.ok ${OBJECTDIR}/_ext/427700826/encx24_mac.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ../../../microchip/tcpip/encx24_mac.c  
	
${OBJECTDIR}/_ext/427700826/file_system.o: ../../../microchip/tcpip/file_system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/file_system.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/file_system.o.ok ${OBJECTDIR}/_ext/427700826/file_system.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/file_system.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/file_system.o.d" -o ${OBJECTDIR}/_ext/427700826/file_system.o ../../../microchip/tcpip/file_system.c  
	
${OBJECTDIR}/_ext/427700826/ftp.o: ../../../microchip/tcpip/ftp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o.ok ${OBJECTDIR}/_ext/427700826/ftp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ftp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/ftp.o.d" -o ${OBJECTDIR}/_ext/427700826/ftp.o ../../../microchip/tcpip/ftp.c  
	
${OBJECTDIR}/_ext/427700826/hash_fnv.o: ../../../microchip/tcpip/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o.ok ${OBJECTDIR}/_ext/427700826/hash_fnv.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ../../../microchip/tcpip/hash_fnv.c  
	
${OBJECTDIR}/_ext/427700826/hash_tbl.o: ../../../microchip/tcpip/hash_tbl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o.ok ${OBJECTDIR}/_ext/427700826/hash_tbl.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ../../../microchip/tcpip/hash_tbl.c  
	
${OBJECTDIR}/_ext/427700826/http2.o: ../../../microchip/tcpip/http2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o.ok ${OBJECTDIR}/_ext/427700826/http2.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/http2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/http2.o.d" -o ${OBJECTDIR}/_ext/427700826/http2.o ../../../microchip/tcpip/http2.c  
	
${OBJECTDIR}/_ext/427700826/icmp.o: ../../../microchip/tcpip/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o.ok ${OBJECTDIR}/_ext/427700826/icmp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/icmp.o.d" -o ${OBJECTDIR}/_ext/427700826/icmp.o ../../../microchip/tcpip/icmp.c  
	
${OBJECTDIR}/_ext/427700826/icmpv6.o: ../../../microchip/tcpip/icmpv6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o.ok ${OBJECTDIR}/_ext/427700826/icmpv6.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" -o ${OBJECTDIR}/_ext/427700826/icmpv6.o ../../../microchip/tcpip/icmpv6.c  
	
${OBJECTDIR}/_ext/427700826/ip.o: ../../../microchip/tcpip/ip.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ip.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ip.o.ok ${OBJECTDIR}/_ext/427700826/ip.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ip.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/ip.o.d" -o ${OBJECTDIR}/_ext/427700826/ip.o ../../../microchip/tcpip/ip.c  
	
${OBJECTDIR}/_ext/427700826/mpfs2.o: ../../../microchip/tcpip/mpfs2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/mpfs2.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/mpfs2.o.ok ${OBJECTDIR}/_ext/427700826/mpfs2.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/mpfs2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/mpfs2.o.d" -o ${OBJECTDIR}/_ext/427700826/mpfs2.o ../../../microchip/tcpip/mpfs2.c  
	
${OBJECTDIR}/_ext/427700826/nbns.o: ../../../microchip/tcpip/nbns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o.ok ${OBJECTDIR}/_ext/427700826/nbns.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/nbns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/nbns.o.d" -o ${OBJECTDIR}/_ext/427700826/nbns.o ../../../microchip/tcpip/nbns.c  
	
${OBJECTDIR}/_ext/427700826/ndp.o: ../../../microchip/tcpip/ndp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o.ok ${OBJECTDIR}/_ext/427700826/ndp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ndp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/ndp.o.d" -o ${OBJECTDIR}/_ext/427700826/ndp.o ../../../microchip/tcpip/ndp.c  
	
${OBJECTDIR}/_ext/427700826/reboot.o: ../../../microchip/tcpip/reboot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/reboot.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/reboot.o.ok ${OBJECTDIR}/_ext/427700826/reboot.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/reboot.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/reboot.o.d" -o ${OBJECTDIR}/_ext/427700826/reboot.o ../../../microchip/tcpip/reboot.c  
	
${OBJECTDIR}/_ext/427700826/rsa.o: ../../../microchip/tcpip/rsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o.ok ${OBJECTDIR}/_ext/427700826/rsa.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/rsa.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/rsa.o.d" -o ${OBJECTDIR}/_ext/427700826/rsa.o ../../../microchip/tcpip/rsa.c  
	
${OBJECTDIR}/_ext/427700826/smtp.o: ../../../microchip/tcpip/smtp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o.ok ${OBJECTDIR}/_ext/427700826/smtp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/smtp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/smtp.o.d" -o ${OBJECTDIR}/_ext/427700826/smtp.o ../../../microchip/tcpip/smtp.c  
	
${OBJECTDIR}/_ext/427700826/snmp.o: ../../../microchip/tcpip/snmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o.ok ${OBJECTDIR}/_ext/427700826/snmp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/snmp.o.d" -o ${OBJECTDIR}/_ext/427700826/snmp.o ../../../microchip/tcpip/snmp.c  
	
${OBJECTDIR}/_ext/427700826/snmpv3.o: ../../../microchip/tcpip/snmpv3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3.o.ok ${OBJECTDIR}/_ext/427700826/snmpv3.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmpv3.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/snmpv3.o.d" -o ${OBJECTDIR}/_ext/427700826/snmpv3.o ../../../microchip/tcpip/snmpv3.c  
	
${OBJECTDIR}/_ext/427700826/snmpv3_usm.o: ../../../microchip/tcpip/snmpv3_usm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.ok ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/snmpv3_usm.o.d" -o ${OBJECTDIR}/_ext/427700826/snmpv3_usm.o ../../../microchip/tcpip/snmpv3_usm.c  
	
${OBJECTDIR}/_ext/427700826/sntp.o: ../../../microchip/tcpip/sntp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o.ok ${OBJECTDIR}/_ext/427700826/sntp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/sntp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/sntp.o.d" -o ${OBJECTDIR}/_ext/427700826/sntp.o ../../../microchip/tcpip/sntp.c  
	
${OBJECTDIR}/_ext/427700826/spi_eeprom.o: ../../../microchip/tcpip/spi_eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.ok ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_eeprom.o ../../../microchip/tcpip/spi_eeprom.c  
	
${OBJECTDIR}/_ext/427700826/spi_flash.o: ../../../microchip/tcpip/spi_flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_flash.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_flash.o.ok ${OBJECTDIR}/_ext/427700826/spi_flash.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_flash.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_flash.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_flash.o ../../../microchip/tcpip/spi_flash.c  
	
${OBJECTDIR}/_ext/427700826/spi_ram.o: ../../../microchip/tcpip/spi_ram.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_ram.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_ram.o.ok ${OBJECTDIR}/_ext/427700826/spi_ram.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_ram.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_ram.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_ram.o ../../../microchip/tcpip/spi_ram.c  
	
${OBJECTDIR}/_ext/427700826/ssl.o: ../../../microchip/tcpip/ssl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o.ok ${OBJECTDIR}/_ext/427700826/ssl.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ssl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/ssl.o.d" -o ${OBJECTDIR}/_ext/427700826/ssl.o ../../../microchip/tcpip/ssl.c  
	
${OBJECTDIR}/_ext/427700826/tcp.o: ../../../microchip/tcpip/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o.ok ${OBJECTDIR}/_ext/427700826/tcp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp.o ../../../microchip/tcpip/tcp.c  
	
${OBJECTDIR}/_ext/427700826/tcp_performance_test.o: ../../../microchip/tcpip/tcp_performance_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.ok ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o ../../../microchip/tcpip/tcp_performance_test.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o: ../../../microchip/tcpip/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ../../../microchip/tcpip/tcpip_heap_alloc.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_helpers.o: ../../../microchip/tcpip/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ../../../microchip/tcpip/tcpip_helpers.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o: ../../../microchip/tcpip/tcpip_mac_object.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624 -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ../../../microchip/tcpip/tcpip_mac_object.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_manager.o: ../../../microchip/tcpip/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ../../../microchip/tcpip/tcpip_manager.c  
	
${OBJECTDIR}/_ext/427700826/tcpip_storage.o: ../../../microchip/tcpip/tcpip_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.ok ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ../../../microchip/tcpip/tcpip_storage.c  
	
${OBJECTDIR}/_ext/427700826/telnet.o: ../../../microchip/tcpip/telnet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o.ok ${OBJECTDIR}/_ext/427700826/telnet.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/telnet.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/telnet.o.d" -o ${OBJECTDIR}/_ext/427700826/telnet.o ../../../microchip/tcpip/telnet.c  
	
${OBJECTDIR}/_ext/427700826/tftpc.o: ../../../microchip/tcpip/tftpc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o.ok ${OBJECTDIR}/_ext/427700826/tftpc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tftpc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/tftpc.o.d" -o ${OBJECTDIR}/_ext/427700826/tftpc.o ../../../microchip/tcpip/tftpc.c  
	
${OBJECTDIR}/_ext/427700826/udp.o: ../../../microchip/tcpip/udp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o.ok ${OBJECTDIR}/_ext/427700826/udp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/427700826/udp.o.d" -o ${OBJECTDIR}/_ext/427700826/udp.o ../../../microchip/tcpip/udp.c  
	
${OBJECTDIR}/_ext/334706090/wf_tx_power.o: ../../../microchip/tcpip/wifi/wf_tx_power.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.ok ${OBJECTDIR}/_ext/334706090/wf_tx_power.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_tx_power.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_tx_power.o ../../../microchip/tcpip/wifi/wf_tx_power.c  
	
${OBJECTDIR}/_ext/334706090/wf_connect.o: ../../../microchip/tcpip/wifi/wf_connect.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connect.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connect.o.ok ${OBJECTDIR}/_ext/334706090/wf_connect.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connect.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connect.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connect.o ../../../microchip/tcpip/wifi/wf_connect.c  
	
${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o: ../../../microchip/tcpip/wifi/wf_connection_algorithm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.ok ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_algorithm.o ../../../microchip/tcpip/wifi/wf_connection_algorithm.c  
	
${OBJECTDIR}/_ext/334706090/wf_connection_manager.o: ../../../microchip/tcpip/wifi/wf_connection_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.ok ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_manager.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_manager.o ../../../microchip/tcpip/wifi/wf_connection_manager.c  
	
${OBJECTDIR}/_ext/334706090/wf_connection_profile.o: ../../../microchip/tcpip/wifi/wf_connection_profile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.ok ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_connection_profile.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_connection_profile.o ../../../microchip/tcpip/wifi/wf_connection_profile.c  
	
${OBJECTDIR}/_ext/334706090/wf_console.o: ../../../microchip/tcpip/wifi/wf_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console.o.ok ${OBJECTDIR}/_ext/334706090/wf_console.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console.o ../../../microchip/tcpip/wifi/wf_console.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_if_config.o: ../../../microchip/tcpip/wifi/wf_console_if_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_if_config.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_if_config.o ../../../microchip/tcpip/wifi/wf_console_if_config.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o: ../../../microchip/tcpip/wifi/wf_console_iw_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_iw_config.o ../../../microchip/tcpip/wifi/wf_console_iw_config.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o: ../../../microchip/tcpip/wifi/wf_console_iw_priv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_iw_priv.o ../../../microchip/tcpip/wifi/wf_console_iw_priv.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o: ../../../microchip/tcpip/wifi/wf_console_msg_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_msg_handler.o ../../../microchip/tcpip/wifi/wf_console_msg_handler.c  
	
${OBJECTDIR}/_ext/334706090/wf_console_msgs.o: ../../../microchip/tcpip/wifi/wf_console_msgs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.ok ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_console_msgs.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_console_msgs.o ../../../microchip/tcpip/wifi/wf_console_msgs.c  
	
${OBJECTDIR}/_ext/334706090/wf_data_txrx.o: ../../../microchip/tcpip/wifi/wf_data_txrx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.ok ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_data_txrx.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_data_txrx.o ../../../microchip/tcpip/wifi/wf_data_txrx.c  
	
${OBJECTDIR}/_ext/334706090/wf_driver_com.o: ../../../microchip/tcpip/wifi/wf_driver_com.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_com.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_com.o.ok ${OBJECTDIR}/_ext/334706090/wf_driver_com.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_driver_com.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_driver_com.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_driver_com.o ../../../microchip/tcpip/wifi/wf_driver_com.c  
	
${OBJECTDIR}/_ext/334706090/wf_driver_raw.o: ../../../microchip/tcpip/wifi/wf_driver_raw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.ok ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_driver_raw.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_driver_raw.o ../../../microchip/tcpip/wifi/wf_driver_raw.c  
	
${OBJECTDIR}/_ext/334706090/wf_easy_config.o: ../../../microchip/tcpip/wifi/wf_easy_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.ok ${OBJECTDIR}/_ext/334706090/wf_easy_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_easy_config.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_easy_config.o ../../../microchip/tcpip/wifi/wf_easy_config.c  
	
${OBJECTDIR}/_ext/334706090/wf_eint.o: ../../../microchip/tcpip/wifi/wf_eint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_eint.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_eint.o.ok ${OBJECTDIR}/_ext/334706090/wf_eint.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_eint.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_eint.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_eint.o ../../../microchip/tcpip/wifi/wf_eint.c  
	
${OBJECTDIR}/_ext/334706090/wf_event_handler.o: ../../../microchip/tcpip/wifi/wf_event_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.ok ${OBJECTDIR}/_ext/334706090/wf_event_handler.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_event_handler.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_event_handler.o ../../../microchip/tcpip/wifi/wf_event_handler.c  
	
${OBJECTDIR}/_ext/334706090/wf_init.o: ../../../microchip/tcpip/wifi/wf_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_init.o.ok ${OBJECTDIR}/_ext/334706090/wf_init.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_init.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_init.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_init.o ../../../microchip/tcpip/wifi/wf_init.c  
	
${OBJECTDIR}/_ext/334706090/wf_mac.o: ../../../microchip/tcpip/wifi/wf_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mac.o.ok ${OBJECTDIR}/_ext/334706090/wf_mac.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_mac.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_mac.o ../../../microchip/tcpip/wifi/wf_mac.c  
	
${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o: ../../../microchip/tcpip/wifi/wf_mgmt_msg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.ok ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_mgmt_msg.o ../../../microchip/tcpip/wifi/wf_mgmt_msg.c  
	
${OBJECTDIR}/_ext/334706090/wf_param_msg.o: ../../../microchip/tcpip/wifi/wf_param_msg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_param_msg.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_param_msg.o.ok ${OBJECTDIR}/_ext/334706090/wf_param_msg.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_param_msg.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_param_msg.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_param_msg.o ../../../microchip/tcpip/wifi/wf_param_msg.c  
	
${OBJECTDIR}/_ext/334706090/wf_power_save.o: ../../../microchip/tcpip/wifi/wf_power_save.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_power_save.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_power_save.o.ok ${OBJECTDIR}/_ext/334706090/wf_power_save.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_power_save.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_power_save.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_power_save.o ../../../microchip/tcpip/wifi/wf_power_save.c  
	
${OBJECTDIR}/_ext/334706090/wf_scan.o: ../../../microchip/tcpip/wifi/wf_scan.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_scan.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_scan.o.ok ${OBJECTDIR}/_ext/334706090/wf_scan.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_scan.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_scan.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_scan.o ../../../microchip/tcpip/wifi/wf_scan.c  
	
${OBJECTDIR}/_ext/334706090/wf_spi.o: ../../../microchip/tcpip/wifi/wf_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/wf_spi.o.ok ${OBJECTDIR}/_ext/334706090/wf_spi.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/wf_spi.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/wf_spi.o.d" -o ${OBJECTDIR}/_ext/334706090/wf_spi.o ../../../microchip/tcpip/wifi/wf_spi.c  
	
${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o: ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.ok ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o.d" -o ${OBJECTDIR}/_ext/334706090/mrf24w_mac_pic32.o ../../../microchip/tcpip/wifi/mrf24w_mac_pic32.c  
	
${OBJECTDIR}/_ext/334706090/mrf24w_events.o: ../../../microchip/tcpip/wifi/mrf24w_events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/334706090 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d 
	@${RM} ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.ok ${OBJECTDIR}/_ext/334706090/mrf24w_events.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/334706090/mrf24w_events.o.d" -o ${OBJECTDIR}/_ext/334706090/mrf24w_events.o ../../../microchip/tcpip/wifi/mrf24w_events.c  
	
${OBJECTDIR}/_ext/101875047/hashes.o: ../../../microchip/common/hashes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o.ok ${OBJECTDIR}/_ext/101875047/hashes.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/hashes.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/101875047/hashes.o.d" -o ${OBJECTDIR}/_ext/101875047/hashes.o ../../../microchip/common/hashes.c  
	
${OBJECTDIR}/_ext/101875047/lfsr.o: ../../../microchip/common/lfsr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o.ok ${OBJECTDIR}/_ext/101875047/lfsr.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/lfsr.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/101875047/lfsr.o.d" -o ${OBJECTDIR}/_ext/101875047/lfsr.o ../../../microchip/common/lfsr.c  
	
${OBJECTDIR}/_ext/365611741/system_random.o: ../../../microchip/system/system_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o.ok ${OBJECTDIR}/_ext/365611741/system_random.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_random.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/365611741/system_random.o.d" -o ${OBJECTDIR}/_ext/365611741/system_random.o ../../../microchip/system/system_random.c  
	
${OBJECTDIR}/_ext/792872985/lcd.o: ../../../microchip/system/drivers/lcd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o.ok ${OBJECTDIR}/_ext/792872985/lcd.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/lcd.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/792872985/lcd.o.d" -o ${OBJECTDIR}/_ext/792872985/lcd.o ../../../microchip/system/drivers/lcd.c  
	
${OBJECTDIR}/_ext/792872985/usart.o: ../../../microchip/system/drivers/usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o.ok ${OBJECTDIR}/_ext/792872985/usart.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/usart.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/792872985/usart.o.d" -o ${OBJECTDIR}/_ext/792872985/usart.o ../../../microchip/system/drivers/usart.c  
	
${OBJECTDIR}/_ext/101875047/big_int.o: ../../../microchip/common/big_int.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o.ok ${OBJECTDIR}/_ext/101875047/big_int.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int.o.d" -o ${OBJECTDIR}/_ext/101875047/big_int.o ../../../microchip/common/big_int.c  
	
${OBJECTDIR}/_ext/365611741/system_debug.o: ../../../microchip/system/system_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o.ok ${OBJECTDIR}/_ext/365611741/system_debug.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_debug.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/365611741/system_debug.o.d" -o ${OBJECTDIR}/_ext/365611741/system_debug.o ../../../microchip/system/system_debug.c  
	
${OBJECTDIR}/_ext/792872985/drv_spi.o: ../../../microchip/system/drivers/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o.ok ${OBJECTDIR}/_ext/792872985/drv_spi.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" -o ${OBJECTDIR}/_ext/792872985/drv_spi.o ../../../microchip/system/drivers/drv_spi.c  
	
${OBJECTDIR}/_ext/999769920/bsp.o: ../../../bsp/explorer16/pic24/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/999769920 
	@${RM} ${OBJECTDIR}/_ext/999769920/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/999769920/bsp.o.ok ${OBJECTDIR}/_ext/999769920/bsp.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/999769920/bsp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DC30_EX16_ENC624_WIFI -I"../configs/c30_expl16/bsp_profile/enc624_mrf24w" -I"../configs/c30_expl16/tcpip_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/tcpip/wifi" -I"../../../microchip/include/common" -mlarge-code -mlarge-data -mconst-in-data -Os -MMD -MF "${OBJECTDIR}/_ext/999769920/bsp.o.d" -o ${OBJECTDIR}/_ext/999769920/bsp.o ../../../bsp/explorer16/pic24/bsp.c  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -o dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}        -Wl,--defsym=__MPLAB_BUILD=1,--heap=8500,--stack=500,-L"..",-Map="${DISTDIR}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}        -Wl,--defsym=__MPLAB_BUILD=1,--heap=8500,--stack=500,-L"..",-Map="${DISTDIR}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}\\pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_c30_e16_enc624_mrf24w_web_server_demo_app.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
endif

.pre:
	@echo "--------------------------------------"
	@echo "User defined pre-build step: []"
	@
	@echo "--------------------------------------"

# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
