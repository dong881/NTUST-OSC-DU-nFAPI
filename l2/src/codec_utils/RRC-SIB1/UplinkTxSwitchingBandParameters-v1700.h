/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UplinkTxSwitchingBandParameters_v1700_H_
#define	_UplinkTxSwitchingBandParameters_v1700_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UplinkTxSwitchingBandParameters_v1700__uplinkTxSwitching2T2T_PUSCH_TransCoherence_r17 {
	UplinkTxSwitchingBandParameters_v1700__uplinkTxSwitching2T2T_PUSCH_TransCoherence_r17_nonCoherent	= 0,
	UplinkTxSwitchingBandParameters_v1700__uplinkTxSwitching2T2T_PUSCH_TransCoherence_r17_fullCoherent	= 1
} e_UplinkTxSwitchingBandParameters_v1700__uplinkTxSwitching2T2T_PUSCH_TransCoherence_r17;

/* UplinkTxSwitchingBandParameters-v1700 */
typedef struct UplinkTxSwitchingBandParameters_v1700 {
	long	 bandIndex_r17;
	long	*uplinkTxSwitching2T2T_PUSCH_TransCoherence_r17;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UplinkTxSwitchingBandParameters_v1700_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_uplinkTxSwitching2T2T_PUSCH_TransCoherence_r17_3;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_UplinkTxSwitchingBandParameters_v1700;
extern asn_SEQUENCE_specifics_t asn_SPC_UplinkTxSwitchingBandParameters_v1700_specs_1;
extern asn_TYPE_member_t asn_MBR_UplinkTxSwitchingBandParameters_v1700_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _UplinkTxSwitchingBandParameters_v1700_H_ */
#include <asn_internal.h>
