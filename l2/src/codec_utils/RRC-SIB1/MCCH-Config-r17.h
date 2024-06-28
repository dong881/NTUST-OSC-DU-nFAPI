/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MCCH_Config_r17_H_
#define	_MCCH_Config_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MCCH-RepetitionPeriodAndOffset-r17.h"
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MCCH_Config_r17__mcch_WindowDuration_r17 {
	MCCH_Config_r17__mcch_WindowDuration_r17_sl2	= 0,
	MCCH_Config_r17__mcch_WindowDuration_r17_sl4	= 1,
	MCCH_Config_r17__mcch_WindowDuration_r17_sl8	= 2,
	MCCH_Config_r17__mcch_WindowDuration_r17_sl10	= 3,
	MCCH_Config_r17__mcch_WindowDuration_r17_sl20	= 4,
	MCCH_Config_r17__mcch_WindowDuration_r17_sl40	= 5,
	MCCH_Config_r17__mcch_WindowDuration_r17_sl80	= 6,
	MCCH_Config_r17__mcch_WindowDuration_r17_sl160	= 7
} e_MCCH_Config_r17__mcch_WindowDuration_r17;
typedef enum MCCH_Config_r17__mcch_ModificationPeriod_r17 {
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf2	= 0,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf4	= 1,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf8	= 2,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf16	= 3,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf32	= 4,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf64	= 5,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf128	= 6,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf256	= 7,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf512	= 8,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf1024	= 9,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_r2048	= 10,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf4096	= 11,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf8192	= 12,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf16384	= 13,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf32768	= 14,
	MCCH_Config_r17__mcch_ModificationPeriod_r17_rf65536	= 15
} e_MCCH_Config_r17__mcch_ModificationPeriod_r17;

/* MCCH-Config-r17 */
typedef struct MCCH_Config_r17 {
	MCCH_RepetitionPeriodAndOffset_r17_t	 mcch_RepetitionPeriodAndOffset_r17;
	long	 mcch_WindowStartSlot_r17;
	long	*mcch_WindowDuration_r17;	/* OPTIONAL */
	long	 mcch_ModificationPeriod_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MCCH_Config_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_mcch_WindowDuration_r17_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mcch_ModificationPeriod_r17_13;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MCCH_Config_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_MCCH_Config_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_MCCH_Config_r17_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _MCCH_Config_r17_H_ */
#include <asn_internal.h>
