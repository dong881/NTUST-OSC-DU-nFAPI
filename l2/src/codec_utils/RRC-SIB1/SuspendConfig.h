/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SuspendConfig_H_
#define	_SuspendConfig_H_


#include <asn_application.h>

/* Including external dependencies */
#include "I-RNTI-Value.h"
#include "ShortI-RNTI-Value.h"
#include "PagingCycle.h"
#include "PeriodicRNAU-TimerValue.h"
#include "NextHopChainingCount.h"
#include "RNTI-Value.h"
#include "ExtendedPagingCycle-r17.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RAN_NotificationAreaInfo;
struct SetupRelease_SDT_Config_r17;
struct SetupRelease_SRS_PosRRC_Inactive_r17;

/* SuspendConfig */
typedef struct SuspendConfig {
	I_RNTI_Value_t	 fullI_RNTI;
	ShortI_RNTI_Value_t	 shortI_RNTI;
	PagingCycle_t	 ran_PagingCycle;
	struct RAN_NotificationAreaInfo	*ran_NotificationAreaInfo;	/* OPTIONAL */
	PeriodicRNAU_TimerValue_t	*t380;	/* OPTIONAL */
	NextHopChainingCount_t	 nextHopChainingCount;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct SuspendConfig__ext1 {
		RNTI_Value_t	*sl_UEIdentityRemote_r17;	/* OPTIONAL */
		struct SetupRelease_SDT_Config_r17	*sdt_Config_r17;	/* OPTIONAL */
		struct SetupRelease_SRS_PosRRC_Inactive_r17	*srs_PosRRC_Inactive_r17;	/* OPTIONAL */
		ExtendedPagingCycle_r17_t	*ran_ExtendedPagingCycle_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SuspendConfig_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SuspendConfig;
extern asn_SEQUENCE_specifics_t asn_SPC_SuspendConfig_specs_1;
extern asn_TYPE_member_t asn_MBR_SuspendConfig_1[7];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RAN-NotificationAreaInfo.h"
#include "SetupRelease.h"

#endif	/* _SuspendConfig_H_ */
#include <asn_internal.h>
