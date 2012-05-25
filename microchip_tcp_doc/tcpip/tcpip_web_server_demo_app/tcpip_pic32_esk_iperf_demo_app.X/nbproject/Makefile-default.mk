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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/979503464/bsp.o ${OBJECTDIR}/_ext/101875047/lfsr.o ${OBJECTDIR}/_ext/101875047/hashes.o ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o ${OBJECTDIR}/_ext/101875047/big_int.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ${OBJECTDIR}/_ext/1472/ping_demo.o ${OBJECTDIR}/_ext/1472/smtp_demo.o ${OBJECTDIR}/_ext/1472/iperf_app.o ${OBJECTDIR}/_ext/1472/iperf_console.o ${OBJECTDIR}/_ext/365611741/system_services.o ${OBJECTDIR}/_ext/365611741/system_debug.o ${OBJECTDIR}/_ext/792872985/usart.o ${OBJECTDIR}/_ext/792872985/lcd.o ${OBJECTDIR}/_ext/365611741/system_random.o ${OBJECTDIR}/_ext/792872985/drv_spi.o ${OBJECTDIR}/_ext/427700826/announce.o ${OBJECTDIR}/_ext/427700826/http2.o ${OBJECTDIR}/_ext/427700826/arcfour.o ${OBJECTDIR}/_ext/427700826/arp.o ${OBJECTDIR}/_ext/427700826/auto_ip.o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ${OBJECTDIR}/_ext/427700826/dhcp.o ${OBJECTDIR}/_ext/427700826/dhcps.o ${OBJECTDIR}/_ext/427700826/dns.o ${OBJECTDIR}/_ext/427700826/dnss.o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ${OBJECTDIR}/_ext/427700826/enc28j60.o ${OBJECTDIR}/_ext/427700826/encs24j600.o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o ${OBJECTDIR}/_ext/427700826/ftp.o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ${OBJECTDIR}/_ext/427700826/spi_eeprom.o ${OBJECTDIR}/_ext/427700826/icmp.o ${OBJECTDIR}/_ext/427700826/icmpv6.o ${OBJECTDIR}/_ext/427700826/ip.o ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o ${OBJECTDIR}/_ext/427700826/mpfs2.o ${OBJECTDIR}/_ext/427700826/nbns.o ${OBJECTDIR}/_ext/427700826/ndp.o ${OBJECTDIR}/_ext/427700826/reboot.o ${OBJECTDIR}/_ext/427700826/rsa.o ${OBJECTDIR}/_ext/427700826/smtp.o ${OBJECTDIR}/_ext/427700826/snmp.o ${OBJECTDIR}/_ext/427700826/sntp.o ${OBJECTDIR}/_ext/427700826/spi_flash.o ${OBJECTDIR}/_ext/427700826/spi_ram.o ${OBJECTDIR}/_ext/427700826/ssl.o ${OBJECTDIR}/_ext/427700826/tcp.o ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ${OBJECTDIR}/_ext/427700826/telnet.o ${OBJECTDIR}/_ext/427700826/tftpc.o ${OBJECTDIR}/_ext/427700826/udp.o ${OBJECTDIR}/_ext/427700826/udp_performance_test.o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ${OBJECTDIR}/_ext/1472/main_demo.o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ${OBJECTDIR}/_ext/1472/mpfs_img2.o ${OBJECTDIR}/_ext/1472/custom_http_app.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/979503464/bsp.o.d ${OBJECTDIR}/_ext/101875047/lfsr.o.d ${OBJECTDIR}/_ext/101875047/hashes.o.d ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d ${OBJECTDIR}/_ext/101875047/big_int.o.d ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d ${OBJECTDIR}/_ext/1472/ping_demo.o.d ${OBJECTDIR}/_ext/1472/smtp_demo.o.d ${OBJECTDIR}/_ext/1472/iperf_app.o.d ${OBJECTDIR}/_ext/1472/iperf_console.o.d ${OBJECTDIR}/_ext/365611741/system_services.o.d ${OBJECTDIR}/_ext/365611741/system_debug.o.d ${OBJECTDIR}/_ext/792872985/usart.o.d ${OBJECTDIR}/_ext/792872985/lcd.o.d ${OBJECTDIR}/_ext/365611741/system_random.o.d ${OBJECTDIR}/_ext/792872985/drv_spi.o.d ${OBJECTDIR}/_ext/427700826/announce.o.d ${OBJECTDIR}/_ext/427700826/http2.o.d ${OBJECTDIR}/_ext/427700826/arcfour.o.d ${OBJECTDIR}/_ext/427700826/arp.o.d ${OBJECTDIR}/_ext/427700826/auto_ip.o.d ${OBJECTDIR}/_ext/427700826/berkeley_api.o.d ${OBJECTDIR}/_ext/427700826/dhcp.o.d ${OBJECTDIR}/_ext/427700826/dhcps.o.d ${OBJECTDIR}/_ext/427700826/dns.o.d ${OBJECTDIR}/_ext/427700826/dnss.o.d ${OBJECTDIR}/_ext/427700826/dyn_dns.o.d ${OBJECTDIR}/_ext/427700826/enc28_mac.o.d ${OBJECTDIR}/_ext/427700826/enc28j60.o.d ${OBJECTDIR}/_ext/427700826/encs24j600.o.d ${OBJECTDIR}/_ext/427700826/encx24_mac.o.d ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d ${OBJECTDIR}/_ext/427700826/ftp.o.d ${OBJECTDIR}/_ext/427700826/hash_fnv.o.d ${OBJECTDIR}/_ext/427700826/hash_tbl.o.d ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d ${OBJECTDIR}/_ext/427700826/icmp.o.d ${OBJECTDIR}/_ext/427700826/icmpv6.o.d ${OBJECTDIR}/_ext/427700826/ip.o.d ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d ${OBJECTDIR}/_ext/427700826/mpfs2.o.d ${OBJECTDIR}/_ext/427700826/nbns.o.d ${OBJECTDIR}/_ext/427700826/ndp.o.d ${OBJECTDIR}/_ext/427700826/reboot.o.d ${OBJECTDIR}/_ext/427700826/rsa.o.d ${OBJECTDIR}/_ext/427700826/smtp.o.d ${OBJECTDIR}/_ext/427700826/snmp.o.d ${OBJECTDIR}/_ext/427700826/sntp.o.d ${OBJECTDIR}/_ext/427700826/spi_flash.o.d ${OBJECTDIR}/_ext/427700826/spi_ram.o.d ${OBJECTDIR}/_ext/427700826/ssl.o.d ${OBJECTDIR}/_ext/427700826/tcp.o.d ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d ${OBJECTDIR}/_ext/427700826/telnet.o.d ${OBJECTDIR}/_ext/427700826/tftpc.o.d ${OBJECTDIR}/_ext/427700826/udp.o.d ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d ${OBJECTDIR}/_ext/1472/main_demo.o.d ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d ${OBJECTDIR}/_ext/1472/mpfs_img2.o.d ${OBJECTDIR}/_ext/1472/custom_http_app.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/979503464/bsp.o ${OBJECTDIR}/_ext/101875047/lfsr.o ${OBJECTDIR}/_ext/101875047/hashes.o ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o ${OBJECTDIR}/_ext/101875047/big_int.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ${OBJECTDIR}/_ext/1472/ping_demo.o ${OBJECTDIR}/_ext/1472/smtp_demo.o ${OBJECTDIR}/_ext/1472/iperf_app.o ${OBJECTDIR}/_ext/1472/iperf_console.o ${OBJECTDIR}/_ext/365611741/system_services.o ${OBJECTDIR}/_ext/365611741/system_debug.o ${OBJECTDIR}/_ext/792872985/usart.o ${OBJECTDIR}/_ext/792872985/lcd.o ${OBJECTDIR}/_ext/365611741/system_random.o ${OBJECTDIR}/_ext/792872985/drv_spi.o ${OBJECTDIR}/_ext/427700826/announce.o ${OBJECTDIR}/_ext/427700826/http2.o ${OBJECTDIR}/_ext/427700826/arcfour.o ${OBJECTDIR}/_ext/427700826/arp.o ${OBJECTDIR}/_ext/427700826/auto_ip.o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ${OBJECTDIR}/_ext/427700826/dhcp.o ${OBJECTDIR}/_ext/427700826/dhcps.o ${OBJECTDIR}/_ext/427700826/dns.o ${OBJECTDIR}/_ext/427700826/dnss.o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ${OBJECTDIR}/_ext/427700826/enc28j60.o ${OBJECTDIR}/_ext/427700826/encs24j600.o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o ${OBJECTDIR}/_ext/427700826/ftp.o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ${OBJECTDIR}/_ext/427700826/spi_eeprom.o ${OBJECTDIR}/_ext/427700826/icmp.o ${OBJECTDIR}/_ext/427700826/icmpv6.o ${OBJECTDIR}/_ext/427700826/ip.o ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o ${OBJECTDIR}/_ext/427700826/mpfs2.o ${OBJECTDIR}/_ext/427700826/nbns.o ${OBJECTDIR}/_ext/427700826/ndp.o ${OBJECTDIR}/_ext/427700826/reboot.o ${OBJECTDIR}/_ext/427700826/rsa.o ${OBJECTDIR}/_ext/427700826/smtp.o ${OBJECTDIR}/_ext/427700826/snmp.o ${OBJECTDIR}/_ext/427700826/sntp.o ${OBJECTDIR}/_ext/427700826/spi_flash.o ${OBJECTDIR}/_ext/427700826/spi_ram.o ${OBJECTDIR}/_ext/427700826/ssl.o ${OBJECTDIR}/_ext/427700826/tcp.o ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ${OBJECTDIR}/_ext/427700826/telnet.o ${OBJECTDIR}/_ext/427700826/tftpc.o ${OBJECTDIR}/_ext/427700826/udp.o ${OBJECTDIR}/_ext/427700826/udp_performance_test.o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ${OBJECTDIR}/_ext/1472/main_demo.o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ${OBJECTDIR}/_ext/1472/mpfs_img2.o ${OBJECTDIR}/_ext/1472/custom_http_app.o


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

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o: ../../../microchip/common/big_int_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.ok ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d" "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.asm.d" -t $(SILENT) -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/include/common" -I"../../../microchip/include/system" -I"../../../microchip/include/system/drivers" -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o ../../../microchip/common/big_int_helper_c32.S  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.asm.d",--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-I".."  
	
else
${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o: ../../../microchip/common/big_int_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.ok ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d" "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.asm.d" -t $(SILENT) -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -I"../../../microchip/include/tcpip" -I"../../../microchip/include/common" -I"../../../microchip/include/system" -I"../../../microchip/include/system/drivers" -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o ../../../microchip/common/big_int_helper_c32.S  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/101875047/big_int_helper_c32.o.asm.d",--gdwarf-2,-I".."  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/979503464/bsp.o: ../../../bsp/pic32_esk/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/979503464 
	@${RM} ${OBJECTDIR}/_ext/979503464/bsp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/979503464/bsp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/979503464/bsp.o.d" -o ${OBJECTDIR}/_ext/979503464/bsp.o ../../../bsp/pic32_esk/bsp.c   -G 64 
	
${OBJECTDIR}/_ext/101875047/lfsr.o: ../../../microchip/common/lfsr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/lfsr.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/lfsr.o.d" -o ${OBJECTDIR}/_ext/101875047/lfsr.o ../../../microchip/common/lfsr.c   -G 64 
	
${OBJECTDIR}/_ext/101875047/hashes.o: ../../../microchip/common/hashes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/hashes.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/hashes.o.d" -o ${OBJECTDIR}/_ext/101875047/hashes.o ../../../microchip/common/hashes.c   -G 64 
	
${OBJECTDIR}/_ext/101875047/big_int.o: ../../../microchip/common/big_int.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int.o.d" -o ${OBJECTDIR}/_ext/101875047/big_int.o ../../../microchip/common/big_int.c   -G 64 
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o: ../berkeley_tcp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ../berkeley_tcp_client_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o: ../berkeley_tcp_server_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ../berkeley_tcp_server_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o: ../berkeley_udp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ../berkeley_udp_client_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/generic_tcp_client.o: ../generic_tcp_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ../generic_tcp_client.c   -G 64 
	
${OBJECTDIR}/_ext/1472/generic_tcp_server.o: ../generic_tcp_server.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ../generic_tcp_server.c   -G 64 
	
${OBJECTDIR}/_ext/1472/ping_demo.o: ../ping_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ping_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/ping_demo.o.d" -o ${OBJECTDIR}/_ext/1472/ping_demo.o ../ping_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/smtp_demo.o: ../smtp_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" -o ${OBJECTDIR}/_ext/1472/smtp_demo.o ../smtp_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/iperf_app.o: ../iperf_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/iperf_app.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/iperf_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/iperf_app.o.d" -o ${OBJECTDIR}/_ext/1472/iperf_app.o ../iperf_app.c   -G 64 
	
${OBJECTDIR}/_ext/1472/iperf_console.o: ../iperf_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/iperf_console.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/iperf_console.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/iperf_console.o.d" -o ${OBJECTDIR}/_ext/1472/iperf_console.o ../iperf_console.c   -G 64 
	
${OBJECTDIR}/_ext/365611741/system_services.o: ../../../microchip/system/system_services.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_services.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_services.o.d" -o ${OBJECTDIR}/_ext/365611741/system_services.o ../../../microchip/system/system_services.c   -G 64 
	
${OBJECTDIR}/_ext/365611741/system_debug.o: ../../../microchip/system/system_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_debug.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_debug.o.d" -o ${OBJECTDIR}/_ext/365611741/system_debug.o ../../../microchip/system/system_debug.c   -G 64 
	
${OBJECTDIR}/_ext/792872985/usart.o: ../../../microchip/system/drivers/usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/usart.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/usart.o.d" -o ${OBJECTDIR}/_ext/792872985/usart.o ../../../microchip/system/drivers/usart.c   -G 64 
	
${OBJECTDIR}/_ext/792872985/lcd.o: ../../../microchip/system/drivers/lcd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/lcd.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/lcd.o.d" -o ${OBJECTDIR}/_ext/792872985/lcd.o ../../../microchip/system/drivers/lcd.c   -G 64 
	
${OBJECTDIR}/_ext/365611741/system_random.o: ../../../microchip/system/system_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_random.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_random.o.d" -o ${OBJECTDIR}/_ext/365611741/system_random.o ../../../microchip/system/system_random.c   -G 64 
	
${OBJECTDIR}/_ext/792872985/drv_spi.o: ../../../microchip/system/drivers/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" -o ${OBJECTDIR}/_ext/792872985/drv_spi.o ../../../microchip/system/drivers/drv_spi.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/announce.o: ../../../microchip/tcpip/announce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/announce.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/announce.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/announce.o.d" -o ${OBJECTDIR}/_ext/427700826/announce.o ../../../microchip/tcpip/announce.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/http2.o: ../../../microchip/tcpip/http2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/http2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/http2.o.d" -o ${OBJECTDIR}/_ext/427700826/http2.o ../../../microchip/tcpip/http2.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/arcfour.o: ../../../microchip/tcpip/arcfour.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arcfour.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/arcfour.o.d" -o ${OBJECTDIR}/_ext/427700826/arcfour.o ../../../microchip/tcpip/arcfour.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/arp.o: ../../../microchip/tcpip/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/arp.o.d" -o ${OBJECTDIR}/_ext/427700826/arp.o ../../../microchip/tcpip/arp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/auto_ip.o: ../../../microchip/tcpip/auto_ip.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/auto_ip.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/auto_ip.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/auto_ip.o.d" -o ${OBJECTDIR}/_ext/427700826/auto_ip.o ../../../microchip/tcpip/auto_ip.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/berkeley_api.o: ../../../microchip/tcpip/berkeley_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" -o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ../../../microchip/tcpip/berkeley_api.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dhcp.o: ../../../microchip/tcpip/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcp.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcp.o ../../../microchip/tcpip/dhcp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dhcps.o: ../../../microchip/tcpip/dhcps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcps.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcps.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcps.o ../../../microchip/tcpip/dhcps.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dns.o: ../../../microchip/tcpip/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dns.o ../../../microchip/tcpip/dns.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dnss.o: ../../../microchip/tcpip/dnss.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dnss.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dnss.o.d" -o ${OBJECTDIR}/_ext/427700826/dnss.o ../../../microchip/tcpip/dnss.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dyn_dns.o: ../../../microchip/tcpip/dyn_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ../../../microchip/tcpip/dyn_dns.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/enc28_mac.o: ../../../microchip/tcpip/enc28_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ../../../microchip/tcpip/enc28_mac.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/enc28j60.o: ../../../microchip/tcpip/enc28j60.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28j60.o ../../../microchip/tcpip/enc28j60.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/encs24j600.o: ../../../microchip/tcpip/encs24j600.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" -o ${OBJECTDIR}/_ext/427700826/encs24j600.o ../../../microchip/tcpip/encs24j600.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/encx24_mac.o: ../../../microchip/tcpip/encx24_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ../../../microchip/tcpip/encx24_mac.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o: ../../../microchip/tcpip/eth_pic32_ext_phy.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o ../../../microchip/tcpip/eth_pic32_ext_phy.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o: ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o: ../../../microchip/tcpip/eth_pic32_int_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o ../../../microchip/tcpip/eth_pic32_int_mac.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/ftp.o: ../../../microchip/tcpip/ftp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ftp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ftp.o.d" -o ${OBJECTDIR}/_ext/427700826/ftp.o ../../../microchip/tcpip/ftp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/hash_fnv.o: ../../../microchip/tcpip/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ../../../microchip/tcpip/hash_fnv.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/hash_tbl.o: ../../../microchip/tcpip/hash_tbl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ../../../microchip/tcpip/hash_tbl.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/spi_eeprom.o: ../../../microchip/tcpip/spi_eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_eeprom.o ../../../microchip/tcpip/spi_eeprom.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/icmp.o: ../../../microchip/tcpip/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/icmp.o.d" -o ${OBJECTDIR}/_ext/427700826/icmp.o ../../../microchip/tcpip/icmp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/icmpv6.o: ../../../microchip/tcpip/icmpv6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" -o ${OBJECTDIR}/_ext/427700826/icmpv6.o ../../../microchip/tcpip/icmpv6.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/ip.o: ../../../microchip/tcpip/ip.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ip.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ip.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ip.o.d" -o ${OBJECTDIR}/_ext/427700826/ip.o ../../../microchip/tcpip/ip.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/mac_events_pic32.o: ../../../microchip/tcpip/mac_events_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d" -o ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o ../../../microchip/tcpip/mac_events_pic32.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/mpfs2.o: ../../../microchip/tcpip/mpfs2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/mpfs2.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/mpfs2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/mpfs2.o.d" -o ${OBJECTDIR}/_ext/427700826/mpfs2.o ../../../microchip/tcpip/mpfs2.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/nbns.o: ../../../microchip/tcpip/nbns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/nbns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/nbns.o.d" -o ${OBJECTDIR}/_ext/427700826/nbns.o ../../../microchip/tcpip/nbns.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/ndp.o: ../../../microchip/tcpip/ndp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ndp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ndp.o.d" -o ${OBJECTDIR}/_ext/427700826/ndp.o ../../../microchip/tcpip/ndp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/reboot.o: ../../../microchip/tcpip/reboot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/reboot.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/reboot.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/reboot.o.d" -o ${OBJECTDIR}/_ext/427700826/reboot.o ../../../microchip/tcpip/reboot.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/rsa.o: ../../../microchip/tcpip/rsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/rsa.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/rsa.o.d" -o ${OBJECTDIR}/_ext/427700826/rsa.o ../../../microchip/tcpip/rsa.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/smtp.o: ../../../microchip/tcpip/smtp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/smtp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/smtp.o.d" -o ${OBJECTDIR}/_ext/427700826/smtp.o ../../../microchip/tcpip/smtp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/snmp.o: ../../../microchip/tcpip/snmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/snmp.o.d" -o ${OBJECTDIR}/_ext/427700826/snmp.o ../../../microchip/tcpip/snmp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/sntp.o: ../../../microchip/tcpip/sntp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/sntp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/sntp.o.d" -o ${OBJECTDIR}/_ext/427700826/sntp.o ../../../microchip/tcpip/sntp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/spi_flash.o: ../../../microchip/tcpip/spi_flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_flash.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_flash.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_flash.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_flash.o ../../../microchip/tcpip/spi_flash.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/spi_ram.o: ../../../microchip/tcpip/spi_ram.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_ram.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_ram.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_ram.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_ram.o ../../../microchip/tcpip/spi_ram.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/ssl.o: ../../../microchip/tcpip/ssl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ssl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ssl.o.d" -o ${OBJECTDIR}/_ext/427700826/ssl.o ../../../microchip/tcpip/ssl.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcp.o: ../../../microchip/tcpip/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp.o ../../../microchip/tcpip/tcp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcp_performance_test.o: ../../../microchip/tcpip/tcp_performance_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o ../../../microchip/tcpip/tcp_performance_test.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o: ../../../microchip/tcpip/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ../../../microchip/tcpip/tcpip_heap_alloc.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_storage.o: ../../../microchip/tcpip/tcpip_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ../../../microchip/tcpip/tcpip_storage.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/telnet.o: ../../../microchip/tcpip/telnet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/telnet.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/telnet.o.d" -o ${OBJECTDIR}/_ext/427700826/telnet.o ../../../microchip/tcpip/telnet.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tftpc.o: ../../../microchip/tcpip/tftpc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tftpc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tftpc.o.d" -o ${OBJECTDIR}/_ext/427700826/tftpc.o ../../../microchip/tcpip/tftpc.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/udp.o: ../../../microchip/tcpip/udp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/udp.o.d" -o ${OBJECTDIR}/_ext/427700826/udp.o ../../../microchip/tcpip/udp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/udp_performance_test.o: ../../../microchip/tcpip/udp_performance_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d" -o ${OBJECTDIR}/_ext/427700826/udp_performance_test.o ../../../microchip/tcpip/udp_performance_test.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o: ../../../microchip/tcpip/tcpip_mac_object.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ../../../microchip/tcpip/tcpip_mac_object.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_manager.o: ../../../microchip/tcpip/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ../../../microchip/tcpip/tcpip_manager.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_helpers.o: ../../../microchip/tcpip/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ../../../microchip/tcpip/tcpip_helpers.c   -G 64 
	
${OBJECTDIR}/_ext/1472/main_demo.o: ../main_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/main_demo.o.d" -o ${OBJECTDIR}/_ext/1472/main_demo.o ../main_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/custom_snmp_app.o: ../custom_snmp_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ../custom_snmp_app.c   -G 64 
	
${OBJECTDIR}/_ext/1472/custom_ssl_cert.o: ../custom_ssl_cert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" -o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ../custom_ssl_cert.c   -G 64 
	
${OBJECTDIR}/_ext/1472/mpfs_img2.o: ../mpfs_img2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/mpfs_img2.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/mpfs_img2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/mpfs_img2.o.d" -o ${OBJECTDIR}/_ext/1472/mpfs_img2.o ../mpfs_img2.c   -G 64 
	
${OBJECTDIR}/_ext/1472/custom_http_app.o: ../custom_http_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_http_app.o ../custom_http_app.c   -G 64 
	
else
${OBJECTDIR}/_ext/979503464/bsp.o: ../../../bsp/pic32_esk/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/979503464 
	@${RM} ${OBJECTDIR}/_ext/979503464/bsp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/979503464/bsp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/979503464/bsp.o.d" -o ${OBJECTDIR}/_ext/979503464/bsp.o ../../../bsp/pic32_esk/bsp.c   -G 64 
	
${OBJECTDIR}/_ext/101875047/lfsr.o: ../../../microchip/common/lfsr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/lfsr.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/lfsr.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/lfsr.o.d" -o ${OBJECTDIR}/_ext/101875047/lfsr.o ../../../microchip/common/lfsr.c   -G 64 
	
${OBJECTDIR}/_ext/101875047/hashes.o: ../../../microchip/common/hashes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/hashes.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/hashes.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/hashes.o.d" -o ${OBJECTDIR}/_ext/101875047/hashes.o ../../../microchip/common/hashes.c   -G 64 
	
${OBJECTDIR}/_ext/101875047/big_int.o: ../../../microchip/common/big_int.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/101875047 
	@${RM} ${OBJECTDIR}/_ext/101875047/big_int.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/101875047/big_int.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/101875047/big_int.o.d" -o ${OBJECTDIR}/_ext/101875047/big_int.o ../../../microchip/common/big_int.c   -G 64 
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o: ../berkeley_tcp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_client_demo.o ../berkeley_tcp_client_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o: ../berkeley_tcp_server_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_tcp_server_demo.o ../berkeley_tcp_server_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o: ../berkeley_udp_client_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o.d" -o ${OBJECTDIR}/_ext/1472/berkeley_udp_client_demo.o ../berkeley_udp_client_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/generic_tcp_client.o: ../generic_tcp_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_client.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_client.o ../generic_tcp_client.c   -G 64 
	
${OBJECTDIR}/_ext/1472/generic_tcp_server.o: ../generic_tcp_server.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/generic_tcp_server.o.d" -o ${OBJECTDIR}/_ext/1472/generic_tcp_server.o ../generic_tcp_server.c   -G 64 
	
${OBJECTDIR}/_ext/1472/ping_demo.o: ../ping_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ping_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ping_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/ping_demo.o.d" -o ${OBJECTDIR}/_ext/1472/ping_demo.o ../ping_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/smtp_demo.o: ../smtp_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/smtp_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/smtp_demo.o.d" -o ${OBJECTDIR}/_ext/1472/smtp_demo.o ../smtp_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/iperf_app.o: ../iperf_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/iperf_app.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/iperf_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/iperf_app.o.d" -o ${OBJECTDIR}/_ext/1472/iperf_app.o ../iperf_app.c   -G 64 
	
${OBJECTDIR}/_ext/1472/iperf_console.o: ../iperf_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/iperf_console.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/iperf_console.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/iperf_console.o.d" -o ${OBJECTDIR}/_ext/1472/iperf_console.o ../iperf_console.c   -G 64 
	
${OBJECTDIR}/_ext/365611741/system_services.o: ../../../microchip/system/system_services.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_services.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_services.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_services.o.d" -o ${OBJECTDIR}/_ext/365611741/system_services.o ../../../microchip/system/system_services.c   -G 64 
	
${OBJECTDIR}/_ext/365611741/system_debug.o: ../../../microchip/system/system_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_debug.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_debug.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_debug.o.d" -o ${OBJECTDIR}/_ext/365611741/system_debug.o ../../../microchip/system/system_debug.c   -G 64 
	
${OBJECTDIR}/_ext/792872985/usart.o: ../../../microchip/system/drivers/usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/usart.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/usart.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/usart.o.d" -o ${OBJECTDIR}/_ext/792872985/usart.o ../../../microchip/system/drivers/usart.c   -G 64 
	
${OBJECTDIR}/_ext/792872985/lcd.o: ../../../microchip/system/drivers/lcd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/lcd.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/lcd.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/lcd.o.d" -o ${OBJECTDIR}/_ext/792872985/lcd.o ../../../microchip/system/drivers/lcd.c   -G 64 
	
${OBJECTDIR}/_ext/365611741/system_random.o: ../../../microchip/system/system_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/365611741 
	@${RM} ${OBJECTDIR}/_ext/365611741/system_random.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/365611741/system_random.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/365611741/system_random.o.d" -o ${OBJECTDIR}/_ext/365611741/system_random.o ../../../microchip/system/system_random.c   -G 64 
	
${OBJECTDIR}/_ext/792872985/drv_spi.o: ../../../microchip/system/drivers/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/792872985 
	@${RM} ${OBJECTDIR}/_ext/792872985/drv_spi.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/792872985/drv_spi.o.d" -o ${OBJECTDIR}/_ext/792872985/drv_spi.o ../../../microchip/system/drivers/drv_spi.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/announce.o: ../../../microchip/tcpip/announce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/announce.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/announce.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/announce.o.d" -o ${OBJECTDIR}/_ext/427700826/announce.o ../../../microchip/tcpip/announce.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/http2.o: ../../../microchip/tcpip/http2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/http2.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/http2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/http2.o.d" -o ${OBJECTDIR}/_ext/427700826/http2.o ../../../microchip/tcpip/http2.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/arcfour.o: ../../../microchip/tcpip/arcfour.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arcfour.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arcfour.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/arcfour.o.d" -o ${OBJECTDIR}/_ext/427700826/arcfour.o ../../../microchip/tcpip/arcfour.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/arp.o: ../../../microchip/tcpip/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/arp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/arp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/arp.o.d" -o ${OBJECTDIR}/_ext/427700826/arp.o ../../../microchip/tcpip/arp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/auto_ip.o: ../../../microchip/tcpip/auto_ip.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/auto_ip.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/auto_ip.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/auto_ip.o.d" -o ${OBJECTDIR}/_ext/427700826/auto_ip.o ../../../microchip/tcpip/auto_ip.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/berkeley_api.o: ../../../microchip/tcpip/berkeley_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/berkeley_api.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/berkeley_api.o.d" -o ${OBJECTDIR}/_ext/427700826/berkeley_api.o ../../../microchip/tcpip/berkeley_api.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dhcp.o: ../../../microchip/tcpip/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcp.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcp.o ../../../microchip/tcpip/dhcp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dhcps.o: ../../../microchip/tcpip/dhcps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dhcps.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dhcps.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dhcps.o.d" -o ${OBJECTDIR}/_ext/427700826/dhcps.o ../../../microchip/tcpip/dhcps.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dns.o: ../../../microchip/tcpip/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dns.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dns.o ../../../microchip/tcpip/dns.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dnss.o: ../../../microchip/tcpip/dnss.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dnss.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dnss.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dnss.o.d" -o ${OBJECTDIR}/_ext/427700826/dnss.o ../../../microchip/tcpip/dnss.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/dyn_dns.o: ../../../microchip/tcpip/dyn_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/dyn_dns.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/dyn_dns.o.d" -o ${OBJECTDIR}/_ext/427700826/dyn_dns.o ../../../microchip/tcpip/dyn_dns.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/enc28_mac.o: ../../../microchip/tcpip/enc28_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28_mac.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28_mac.o ../../../microchip/tcpip/enc28_mac.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/enc28j60.o: ../../../microchip/tcpip/enc28j60.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/enc28j60.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/enc28j60.o.d" -o ${OBJECTDIR}/_ext/427700826/enc28j60.o ../../../microchip/tcpip/enc28j60.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/encs24j600.o: ../../../microchip/tcpip/encs24j600.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encs24j600.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/encs24j600.o.d" -o ${OBJECTDIR}/_ext/427700826/encs24j600.o ../../../microchip/tcpip/encs24j600.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/encx24_mac.o: ../../../microchip/tcpip/encx24_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/encx24_mac.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/encx24_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/encx24_mac.o ../../../microchip/tcpip/encx24_mac.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o: ../../../microchip/tcpip/eth_pic32_ext_phy.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy.o ../../../microchip/tcpip/eth_pic32_ext_phy.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o: ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_ext_phy_dp83848.o ../../../microchip/tcpip/eth_pic32_ext_phy_dp83848.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o: ../../../microchip/tcpip/eth_pic32_int_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o.d" -o ${OBJECTDIR}/_ext/427700826/eth_pic32_int_mac.o ../../../microchip/tcpip/eth_pic32_int_mac.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/ftp.o: ../../../microchip/tcpip/ftp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ftp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ftp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ftp.o.d" -o ${OBJECTDIR}/_ext/427700826/ftp.o ../../../microchip/tcpip/ftp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/hash_fnv.o: ../../../microchip/tcpip/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_fnv.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_fnv.o ../../../microchip/tcpip/hash_fnv.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/hash_tbl.o: ../../../microchip/tcpip/hash_tbl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/hash_tbl.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/hash_tbl.o.d" -o ${OBJECTDIR}/_ext/427700826/hash_tbl.o ../../../microchip/tcpip/hash_tbl.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/spi_eeprom.o: ../../../microchip/tcpip/spi_eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_eeprom.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_eeprom.o ../../../microchip/tcpip/spi_eeprom.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/icmp.o: ../../../microchip/tcpip/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/icmp.o.d" -o ${OBJECTDIR}/_ext/427700826/icmp.o ../../../microchip/tcpip/icmp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/icmpv6.o: ../../../microchip/tcpip/icmpv6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/icmpv6.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/icmpv6.o.d" -o ${OBJECTDIR}/_ext/427700826/icmpv6.o ../../../microchip/tcpip/icmpv6.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/ip.o: ../../../microchip/tcpip/ip.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ip.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ip.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ip.o.d" -o ${OBJECTDIR}/_ext/427700826/ip.o ../../../microchip/tcpip/ip.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/mac_events_pic32.o: ../../../microchip/tcpip/mac_events_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/mac_events_pic32.o.d" -o ${OBJECTDIR}/_ext/427700826/mac_events_pic32.o ../../../microchip/tcpip/mac_events_pic32.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/mpfs2.o: ../../../microchip/tcpip/mpfs2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/mpfs2.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/mpfs2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/mpfs2.o.d" -o ${OBJECTDIR}/_ext/427700826/mpfs2.o ../../../microchip/tcpip/mpfs2.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/nbns.o: ../../../microchip/tcpip/nbns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/nbns.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/nbns.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/nbns.o.d" -o ${OBJECTDIR}/_ext/427700826/nbns.o ../../../microchip/tcpip/nbns.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/ndp.o: ../../../microchip/tcpip/ndp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ndp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ndp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ndp.o.d" -o ${OBJECTDIR}/_ext/427700826/ndp.o ../../../microchip/tcpip/ndp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/reboot.o: ../../../microchip/tcpip/reboot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/reboot.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/reboot.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/reboot.o.d" -o ${OBJECTDIR}/_ext/427700826/reboot.o ../../../microchip/tcpip/reboot.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/rsa.o: ../../../microchip/tcpip/rsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/rsa.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/rsa.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/rsa.o.d" -o ${OBJECTDIR}/_ext/427700826/rsa.o ../../../microchip/tcpip/rsa.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/smtp.o: ../../../microchip/tcpip/smtp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/smtp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/smtp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/smtp.o.d" -o ${OBJECTDIR}/_ext/427700826/smtp.o ../../../microchip/tcpip/smtp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/snmp.o: ../../../microchip/tcpip/snmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/snmp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/snmp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/snmp.o.d" -o ${OBJECTDIR}/_ext/427700826/snmp.o ../../../microchip/tcpip/snmp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/sntp.o: ../../../microchip/tcpip/sntp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/sntp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/sntp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/sntp.o.d" -o ${OBJECTDIR}/_ext/427700826/sntp.o ../../../microchip/tcpip/sntp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/spi_flash.o: ../../../microchip/tcpip/spi_flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_flash.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_flash.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_flash.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_flash.o ../../../microchip/tcpip/spi_flash.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/spi_ram.o: ../../../microchip/tcpip/spi_ram.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/spi_ram.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/spi_ram.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/spi_ram.o.d" -o ${OBJECTDIR}/_ext/427700826/spi_ram.o ../../../microchip/tcpip/spi_ram.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/ssl.o: ../../../microchip/tcpip/ssl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/ssl.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/ssl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/ssl.o.d" -o ${OBJECTDIR}/_ext/427700826/ssl.o ../../../microchip/tcpip/ssl.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcp.o: ../../../microchip/tcpip/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp.o ../../../microchip/tcpip/tcp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcp_performance_test.o: ../../../microchip/tcpip/tcp_performance_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcp_performance_test.o.d" -o ${OBJECTDIR}/_ext/427700826/tcp_performance_test.o ../../../microchip/tcpip/tcp_performance_test.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o: ../../../microchip/tcpip/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_heap_alloc.o ../../../microchip/tcpip/tcpip_heap_alloc.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_storage.o: ../../../microchip/tcpip/tcpip_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_storage.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_storage.o ../../../microchip/tcpip/tcpip_storage.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/telnet.o: ../../../microchip/tcpip/telnet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/telnet.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/telnet.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/telnet.o.d" -o ${OBJECTDIR}/_ext/427700826/telnet.o ../../../microchip/tcpip/telnet.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tftpc.o: ../../../microchip/tcpip/tftpc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tftpc.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tftpc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tftpc.o.d" -o ${OBJECTDIR}/_ext/427700826/tftpc.o ../../../microchip/tcpip/tftpc.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/udp.o: ../../../microchip/tcpip/udp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/udp.o.d" -o ${OBJECTDIR}/_ext/427700826/udp.o ../../../microchip/tcpip/udp.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/udp_performance_test.o: ../../../microchip/tcpip/udp_performance_test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/udp_performance_test.o.d" -o ${OBJECTDIR}/_ext/427700826/udp_performance_test.o ../../../microchip/tcpip/udp_performance_test.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o: ../../../microchip/tcpip/tcpip_mac_object.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_mac_object.o ../../../microchip/tcpip/tcpip_mac_object.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_manager.o: ../../../microchip/tcpip/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_manager.o ../../../microchip/tcpip/tcpip_manager.c   -G 64 
	
${OBJECTDIR}/_ext/427700826/tcpip_helpers.o: ../../../microchip/tcpip/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/427700826 
	@${RM} ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/427700826/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/427700826/tcpip_helpers.o ../../../microchip/tcpip/tcpip_helpers.c   -G 64 
	
${OBJECTDIR}/_ext/1472/main_demo.o: ../main_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main_demo.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main_demo.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/main_demo.o.d" -o ${OBJECTDIR}/_ext/1472/main_demo.o ../main_demo.c   -G 64 
	
${OBJECTDIR}/_ext/1472/custom_snmp_app.o: ../custom_snmp_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_snmp_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_snmp_app.o ../custom_snmp_app.c   -G 64 
	
${OBJECTDIR}/_ext/1472/custom_ssl_cert.o: ../custom_ssl_cert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_ssl_cert.o.d" -o ${OBJECTDIR}/_ext/1472/custom_ssl_cert.o ../custom_ssl_cert.c   -G 64 
	
${OBJECTDIR}/_ext/1472/mpfs_img2.o: ../mpfs_img2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/mpfs_img2.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/mpfs_img2.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/mpfs_img2.o.d" -o ${OBJECTDIR}/_ext/1472/mpfs_img2.o ../mpfs_img2.c   -G 64 
	
${OBJECTDIR}/_ext/1472/custom_http_app.o: ../custom_http_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/custom_http_app.o.d 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -I"../configs/pic32_eth_sk/tcpip_profile" -I"../configs/pic32_eth_sk/bsp_profile" -I".." -I"../../../microchip/include" -I"../../../microchip/tcpip" -Wall -MMD -MF "${OBJECTDIR}/_ext/1472/custom_http_app.o.d" -o ${OBJECTDIR}/_ext/1472/custom_http_app.o ../custom_http_app.c   -G 64 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}        -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--defsym=_min_heap_size=40960,--defsym=_min_stack_size=2048,-L"..",-Map="${DISTDIR}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.map",--gc-sections -Os  
else
dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}        -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=40960,--defsym=_min_stack_size=2048,-L"..",-Map="${DISTDIR}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.map",--gc-sections -Os 
	${MP_CC_DIR}\\pic32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/tcpip_pic32_esk_iperf_demo_app.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  
endif


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
