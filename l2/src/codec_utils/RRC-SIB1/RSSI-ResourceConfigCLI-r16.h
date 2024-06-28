/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_RSSI_ResourceConfigCLI_r16_H_
#define	_RSSI_ResourceConfigCLI_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RSSI-ResourceId-r16.h"
#include "SubcarrierSpacing.h"
#include <NativeInteger.h>
#include "RSSI-PeriodicityAndOffset-r16.h"
#include "ServCellIndex.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RSSI-ResourceConfigCLI-r16 */
typedef struct RSSI_ResourceConfigCLI_r16 {
	RSSI_ResourceId_r16_t	 rssi_ResourceId_r16;
	SubcarrierSpacing_t	 rssi_SCS_r16;
	long	 startPRB_r16;
	long	 nrofPRBs_r16;
	long	 startPosition_r16;
	long	 nrofSymbols_r16;
	RSSI_PeriodicityAndOffset_r16_t	 rssi_PeriodicityAndOffset_r16;
	ServCellIndex_t	*refServCellIndex_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RSSI_ResourceConfigCLI_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RSSI_ResourceConfigCLI_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_RSSI_ResourceConfigCLI_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_RSSI_ResourceConfigCLI_r16_1[8];

#ifdef __cplusplus
}
#endif

#endif	/* _RSSI_ResourceConfigCLI_r16_H_ */
#include <asn_internal.h>
